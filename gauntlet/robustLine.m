x = [];
y = [];
syms x y
global mesh_x mesh_y vplot v_symbolic
[mesh_x,mesh_y]=meshgrid(-1.5:0.1:2.5,-3.37:0.1:1);
vplot = 0
v_symbolic = 0
sub = rossubscriber('/scan');
pub = rospublisher('raw_vel');
sub_bump = rossubscriber('/bump');
sub_encoders = rossubscriber('/encoders');
load gauntlet.mat

% Parameters for the robot to turn
turn_vel = [-.1, .1];
drive_vel = [.2, .2]
wheel_d = 0.235 %wheelbase
omega = 2*abs(turn_vel(1))/wheel_d
% stop the robot if it's going right now
stopMsg = rosmessage(pub);
move = rosmessage(pub);
stopMsg.Data = [0 0];
send(pub, stopMsg);
d = 0.084 % offset of the lidar scanner of the neato

% place Neato at the origin pointing in the ihat_G direction
neato_x = 0
neato_y = 0
neato_phi = 1*pi/2
placeNeato(neato_x,neato_y, cos(neato_phi), sin(neato_phi))
% wait a while for the Neato to fall into place
pause(1);
view_fields = 0
view_ransac = 0
theta_change = [];
distance_change = [];
while 1
    vplot = 0
    v_symbolic = 0
    % Scan with the lidar
    scan_message = receive(sub);
    scan_r = scan_message.Ranges(1:end-1);
    scan_theta = deg2rad([0:359]');
    % if keeping track of position than you can convert to global
%     [xg, yg] = toGlobal(scan_r, scan_theta, neato_x, neato_y, neato_phi, d)
%     plot(xg,yg,'ks')

    % Process the scan(adds potential fields), this is all done relative to
    % the neato now
    process_scan(x, y, scan_r, scan_theta, neato_x, neato_y, neato_phi, d, 1, view_ransac);
    
    % Now calculate the gradient and 
    sym_grad = gradient(v_symbolic)
    steep = -1*vpa(subs(sym_grad,[x,y], [0, 0])) %Robot is always at 0,0 since this is relativs
    delta_angle = vpa(atan2(steep(2), steep(1)))
    
        % visualize the potential fields and gradient
    if view_fields == 1
        % Plot the potential field (gradient)
        sigma = 1;
        lambda = .01;
        r2 = [0,0]; % always 0,0 since relative
        pt_gradient = -vpa(subs(sym_grad,[x y],r2(1,:)));
        cutoff = .01
        count = 0
        while  count < 8 %&&norm(pt_gradient) > .05  % && (gradient(1) > cutoff || gradient(2) > cutoff)
             r_prev = r2(end, :);
             pt_gradient = -vpa(subs(sym_grad,[x y],r_prev));
             r_next = r_prev + lambda.*pt_gradient';
             r2 = [r2;r_next]
             lambda = lambda*sigma;
             count = count +1
        end
        % Plot a one shot gradient descent
        plot(r2(:,1), r2(:,2), '-g')
        % THSI PLOTS ACTUALPATH IF YOU RUN IN RELATIVE FRAME FIRST
        %plot(r(:,1),r(:,2), '--r')
        hold on
        contour(mesh_x,mesh_y,vplot,'k','ShowText','On')
        hold on
        [FX, FY] = gradient(vplot);
        scale = 30;
        quiver(mesh_x,mesh_y,FX/scale,FY/scale, 0);
        axis equal
        ylabel('Y Global Axis (m)')
        xlabel('X Global Axis (m)')
        title('Gauntlet Map, Gradient, Potential Feild Contour and Planned Path')
        hold on
        legend('Detected Line', 'Detected Line','Detected Line','Detected Line','Detected Line','Detected Line','Detected Line','Detected Line', 'Desired Neato Path','Actual Neato Path','Contour Plot', 'Gradient Arrows')
%         figure 
%         mesh(mesh_x,mesh_y,vplot)
%         ylabel('Y Global Axis (m)')
%         xlabel('X Global Axis (m)')
%         title('3D Mesh plot of potential fieds')
    end
    
    % Now make the robot turn!!  
    % Get encoder data
    encoder = receive(sub_encoders,1);
    start_encoder = encoder.Data(1);
    
    cutoff_time = abs(delta_angle)/omega
    if delta_angle < 0
        turn_vel = turn_vel * -1;
    end
    move.Data = turn_vel;
    send(pub, move);
    start = rostime('now');
    while 1 
        curr_time = rostime('now')-start;
        if curr_time.seconds >= cutoff_time
            send(pub, stopMsg);
            break 
        end
    end
    encoder = receive(sub_encoders,1);
    after_encoder = encoder.Data(1);
    encoder_change = after_encoder-start_encoder
    delta_angle = -encoder_change/(pi*.235)*(2*pi) 
    
    theta_change = [theta_change;delta_angle]
    
    %Now we need to drive forward along the gradient some
    distance = norm(steep)/30;
    cutoff_time_drive = distance/(drive_vel(1));
    move.Data = drive_vel;
    send(pub, move);
    start = rostime('now');
    bump_detected = 0;
    % Get encoder data
    encoder = receive(sub_encoders,1);
    start_encoder = encoder.Data(1);
    while 1
        curr_time = rostime('now')-start;
        if curr_time.seconds >= cutoff_time_drive
            send(pub, stopMsg);
            break 
        end
        % Check for bump and stop the robot if you did
        bumpMessage = receive(sub_bump);
        if any(bumpMessage.Data)
            bump_detected = 1
            send(pub, stopMsg);
            break;
        end
    end
    encoder = receive(sub_encoders,1);
    after_encoder = encoder.Data(1);
    encoder_change = after_encoder-start_encoder;
    chg_dist = encoder_change ;
    
    distance_change = [distance_change;chg_dist]
    
    
    % if you hit somethign while driving then time to get out
    if bump_detected == 1
        break
    end
 
end
% Reconstruct the path
% RUN THIS ONCE AND THEN UNCOMMENT LINE TO BE ABLE TO PLOT THIS IN GLOBAL
% FRAME
%     r = zeros(length(theta_change)+1, 2); % x and y position
%     theta = zeros(length(theta_change)+1,1);
%     theta(1) = pi/2 % set the heading correctly to start!
%     % move curve to starting position
%     r(1,1) = 0 %x
%     r(1,2) = 0; %t
%     distance_change = [0;distance_change]
%     for n=1:length(theta_change)+1
%        r(n+1, 1) = r(n, 1) + distance_change(n)*cos(theta(n));
%        r(n+1, 2) = r(n, 2) + distance_change(n)*sin(theta(n));
%        if n ~=length(theta_change)+1
%             theta(n+1) =  theta(n) + theta_change(n);
%        end
%     end
%     % Plot experimental position
%     plot(r(:,1),r(:,2), '--r')
%     axis equal



function placeNeato(posX, posY, headingX, headingY)
    svc = rossvcclient('gazebo/set_model_state');
    msg = rosmessage(svc);

    msg.ModelState.ModelName = 'neato_standalone';
    startYaw = atan2(headingY, headingX)
    quat = eul2quat([startYaw 0 0]);

    msg.ModelState.Pose.Position.X = posX;
    msg.ModelState.Pose.Position.Y = posY;
    msg.ModelState.Pose.Position.Z = 1.0;
    msg.ModelState.Pose.Orientation.W = quat(1);
    msg.ModelState.Pose.Orientation.X = quat(2);
    msg.ModelState.Pose.Orientation.Y = quat(3);
    msg.ModelState.Pose.Orientation.Z = quat(4);

    % put the robot in the appropriate place
    ret = call(svc, msg);
end
function [bestr, bestcenter, bestInlierSet, bestOutlierSet] = robustArcFit(points, d, n, target_r, visualize)
    clear A b w xc yc
     bestr = [];
     bestInlierSet = zeros(0,2);
     bestOutlierSet = zeros(0,2);
     bestcenter = zeros(0,2);
    for k=1:n %number of candidate circles to try

        %The minimum number of points we would need to define a circle is two,
        %but with only two points you have two-solutions for the location of
        %the center. Instead of dealing with that, I am going to sleect three
        %points in this case.
        %Select three points at random using the 'datasample' function
        candidates = datasample(points, 3, 'Replace', false);
        circx =candidates(:,1);
        circy =candidates(:,2);
        %now we apply our linear regression routine to the three points.
        %apply our linear regression approach
        A = [circx circy ones(size(circx))];
        b = -circx.^2 - circy.^2;
        w = A\b;

        % convert from the least squares solution to the more familiar parameters
        % of a circle.
        xc = -w(1)/2;
        yc = -w(2)/2;
        rfit = sqrt(xc.^2 + yc.^2 - w(3));

        %To identify inliers, we are going to look for points that are within a
        %threshold distance from our best fit circle.
        threshold = d;
        distFromCircle = abs(sqrt((points(:,1) - xc).^2 + (points(:,2) - yc).^2) - rfit);
        inliers=distFromCircle < threshold;

        %Now, we check if the number of inliers is greater than the best we
        %have found. If so, the candidate line is our new best candidate. We
        %also want to make sure the fitted radius is close to the radius of the
        %BOB
        if abs(rfit-target_r) < 0.05  && sum(inliers) > size(bestInlierSet,1)
            bestInlierSet=points(inliers,:); %points where logical array is true
            bestOutlierSet = points(~inliers, :); %points where logical array is not true
            bestr=rfit;
            bestcenter=[xc,yc];
        end

    end
    if visualize == 1
        %Visualize the result
        figure
        h1=plot(points(:,1),points(:,2),'ks');
        hold on
        %h2=viscircles([bestcenter(1) bestcenter(2)], bestr);
        h3 = plot(bestInlierSet(:,1), bestInlierSet(:,2),'bs');
        title('Circle Fitting')
        legend([h1 h3],'Scan Points','Fit Circle', 'Inliers')
        xlabel('[m]')
        ylabel('[m]')
    end
end

function [fitline_coefs,bestInlierSet,bestOutlierSet,bestEndPoints]= robustLineFit(points,d,n,visualize)
    %The [fitline_coefs,bestInlierSet,bestOutlierSet,bestEndPoints]= robustLineFit(r,theta,d,n) 
    %function runs the RANSAC algorithm for n candidate lines and a threshold of d. The inputs r and
    %theta are polar coordinates. The output fitline_coefs are the coefficients
    %of the best fit line in the format [m b] where y=m*x+b. If you want
    %to visualize, set visualize flag to 1, off is 0. Default is true.

     if ~exist('visualize','var')
         % visualize parameter does not exist, so default it to 1
          visualize = 1;
     end

    %now let's actually implement the RANSAC algorithm
     bestcandidates = [];
     bestInlierSet = zeros(0,2);
     bestOutlierSet = zeros(0,2);
     bestEndPoints = zeros(0,2);
    for k=1:n %number of candidate lines to try


        %select two points at random using the 'datasample' function to define
        %the endpoints of the first candidate fit line
        candidates = datasample(points, 2, 'Replace', false);

        %Find the vector that points from point 2 to point 1
        v=(candidates(1,:)-candidates(2,:))';

        %Check the length of the vector v. If it is zero, the datasample
        %function chose the same point twice, and we need to resample. The
        %continue command will pass to the next iteration of the for loop.
        if norm(v) == 0
            continue;
        end

        %Determine whether points are outliers, we need to know the
        %perpendicular distance away from the candidate fit line. To do this,
        %we first need to define the perpendicular, or orthogonal, direction.
        orthv= [-v(2); v(1)];
        orthv_unit=orthv/norm(orthv); %make this a unit vector

        %Here, we are finding the distance of each scan point from one of the
        %endpoints of our candidate line. At this point this is not the
        %distance perpendicular to the candidate line.
        diffs = points - candidates(2,:);

        %Next, we need to project the difference vectors above onto the
        %perpendicular direction in 'orthv_unit'. This will give us the
        %orthogonal distances from the canidate fit line.
        orthdists=diffs*orthv_unit;
        %test = dot(diffs(1,:), orthv_unit)
        %To identify inliers, we will look for points at a perpendicular
        %distance from the candidate fit line less than the threshold value.
        %The output will be a logic array, with a 1 if the statement is true
        %and 0 if false.
        inliers=abs(orthdists) < d;

        %we also want to check that there are no big gaps in our walls. To do
        %this, we are first taking the distance of each inlier away from an
        %endpoint (diffs) and projecting onto the best fit direction. We then
        %sort these from smallest to largest and take difference to find the
        %spacing between adjacent points. We then identify the maximum gap.
        biggestGap = max(diff(sort(diffs(inliers,:)*v/norm(v))));

        %Now, we check if the number of inliers is greater than the best we
        %have found. If so, the candidate line is our new best candidate. We
        %also make sure there are no big gaps.
        if biggestGap < 0.2  && sum(inliers) > size(bestInlierSet,1)
    %          if sum(inliers) > size(bestInlierSet,1)
            bestInlierSet=points(inliers,:); %points where logical array is true
            bestOutlierSet = points(~inliers, :); %points where logical array is not true
            bestcandidates=candidates;

            %these two lines find a nice set of endpoints for plotting the best
            %fit line
            projectedCoordinate = diffs(inliers, :)*v/norm(v);
            bestEndPoints = [min(projectedCoordinate); max(projectedCoordinate)]*v'/norm(v) + repmat(candidates(2, :), [2, 1]);
        end

    end

    if isempty(bestEndPoints)
        m= NaN;
        b= NaN;
        bestEndPoints=[NaN,NaN;NaN,NaN];
        fitline_coefs=[m b];
        return;
    end

    %Find the coefficients for the best line
    m=diff(bestEndPoints(:,2))/diff(bestEndPoints(:,1));
    b=bestEndPoints(1,2)-m*bestEndPoints(1,1);
    fitline_coefs=[m b];

    if ~1%visualize==1

        % %plot the polar data as verification
        % figure(1)
        % polarplot(deg2rad(theta_clean),r_clean,'ks','MarkerSize',6,'MarkerFaceColor','m')
        % title('Visualization of Polar Data')

        figure(2)
        plot(points(:,1),points(:,2),'ks')
        title('Scan Data- Clean')
        xlabel('[m]')
        ylabel('[m]')

        %Now we need to plot our results
        figure(3)
        plot(bestInlierSet(:,1), bestInlierSet(:,2), 'ks')
        hold on
        plot(bestOutlierSet(:,1),bestOutlierSet(:,2),'bs')
        plot(bestEndPoints(:,1), bestEndPoints(:,2), 'r')
        legend('Inliers','Outliers','Best Fit','location','northwest')
        title(['RANSAC with d=' num2str(d) ' and n=' num2str(n)])
        xlabel('[m]')
        ylabel('[m]')
        % Create textbox
        annotation(figure(3),'textbox',...
            [0.167071428571429 0.152380952380952 0.25 0.1],...
            'String',{'Number of Inliers:' num2str(size(bestInlierSet,1))},...
            'FitBoxToText','off');
    end
end


function add_potential_line(x, y, points, peak_spacing)
    global mesh_x mesh_y vplot v_symbolic
    x1 = mesh_x;
    y1 = mesh_y;
    
    len = pdist(points);
    steps = len/peak_spacing;
    
    xstep = (points(2,1) - points(1,1))/steps;
    ystep = (points(2,2) - points(1,2))/steps;
    for a = 1:steps
        v_symbolic = v_symbolic -log(sqrt((x-points(1,1)-a*xstep).^2 + (y-points(1,2)-a*ystep).^2));
        vplot = vplot -log(sqrt((x1-points(1,1)-a*xstep).^2 + (y1-points(1,2)-a*ystep).^2));
    end
end

function add_potential_bucket(x,y, bucket_xy, r)
    global mesh_x mesh_y vplot v_symbolic
    x1 = mesh_x;
    y1 = mesh_y;
    for theta = 0:0.05*pi:2*pi
        a = r*cos(theta);
        b = r*sin(theta);
        v_symbolic = v_symbolic + log(sqrt((x-a-bucket_xy(1)).^2 + (y-b-bucket_xy(2)).^2));
        vplot = vplot + log(sqrt((x1-a-bucket_xy(1)).^2 + (y1-b-bucket_xy(2)).^2));
    end
end

function process_scan(x,y, r, theta, neato_x, neato_y, neato_phi, d, relative, visualize)
        %_____________________________
    %////Process the scan
    %_____________________________
    % Clean scan data and pass xy cooridnates to line function
    %eliminate zeros
    index=find(r~=0 & r<3);
    r_clean=r(index);
    theta_clean=theta(index);
    %convert to Cartesian and plot again for verification
    if relative == 1
         x_scan = r_clean.*cos(theta_clean)-d
         y_scan = r_clean.*sin(theta_clean)
        
    else% Convert to cartesian and global frame
        [x_scan, y_scan] = toGlobal(r_clean, theta_clean, neato_x, neato_y, neato_phi, d)
    end
    points=[x_scan,y_scan];

    % Store the fit line equations and the end points for each line
    fit_lines = []
    end_pts = [];
    bucket_pos = []; 
    bucket_r = [];
    max_attempts_or_lines = 60;
    first_run = 1;
    % LOOP THROUGH AND DISTINGUISH LINES, and arcs first
    while length(points) > 5 && max_attempts_or_lines > 0
        % Look for arc or bucket first
        if isempty(bucket_pos)
            first_run = 0
            [bestr, best_bucket_pos, circ_inliers, circ_outliers] = robustArcFit(points, .02, 1000, .25, 0);
            % Do a resample of the innliers
            valid_circle = 1
            if ~isempty(bestr)% if you think you found a circle
                for k=1:10 %number of candidate circles to try
                    candidates = datasample(circ_inliers, 3, 'Replace', false);
                    circx =candidates(:,1);
                    circy =candidates(:,2);
                    %now we apply our linear regression routine to the three points.
                    %apply our linear regression approach
                    A = [circx circy ones(size(circx))];
                    b = -circx.^2 - circy.^2;
                    w = A\b;
                    % convert from the least squares solution to the more familiar parameters
                    % of a circle.
                    xc = -w(1)/2;
                    yc = -w(2)/2;
                    rfit = sqrt(xc.^2 + yc.^2 - w(3));
                    if rfit < .8*bestr || rfit > 1.1*bestr
                       valid_circle = 0
                       break 
                    end
                end
                if valid_circle == 1
                    points = setdiff(points,circ_inliers,'rows');
                    bucket_pos = best_bucket_pos
                    bucket_r = bestr
                end
            end
        end
        [fitLine, inliers, outliers, endPts] = robustLineFit(points, .02, 500, 0);


        % Remove inliers
        if ~isempty(inliers)
            points = setdiff(points,inliers,'rows')
            fit_lines = [fit_lines; fitLine]
            end_pts = [end_pts; endPts]
        end
        max_attempts_or_lines = max_attempts_or_lines - 1
    end

    % Visualize all of the lines that were detected
    if visualize ==1
        figure
        if ~isempty(bucket_r) && ~isempty(bucket_pos)
            viscircles([bucket_pos(1) bucket_pos(2)], bucket_r, 'Color','b');
            axis equal
            hold on
        end
        for i=1:2:length(end_pts)
            plot(end_pts(i:i+1,1),  end_pts(i:i+1,2), 'r')
            hold on
        end
    end

    % Test building a potential field of the bucket of beenvolense
    % CHECK TO MAKE SURE IT HAS VLAUES SO IT DOESN"T GET DRAWN AT ORIGIN
    if ~isempty(bucket_r) && ~isempty(bucket_pos)
        add_potential_bucket(x,y,bucket_pos, bestr);
    end

    for i=1:2:length(end_pts)
       add_potential_line(x, y, end_pts(i:i+1, :), .2)
    end
end

function [xg yg]=toGlobal(r_scan, theta_scan, pos_x, pos_y, phi, x_lidar_off)
    xg = r_scan.*cos(theta_scan +phi)-x_lidar_off*cos(phi)+ pos_x
    yg = r_scan.*sin(theta_scan+ phi)-x_lidar_off*sin(phi)+ pos_y
end