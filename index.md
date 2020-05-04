# Navigating the Gauntet!
## The Challenge
The Gauntlet challenge is to navigate a robot successfully from a starting position in to the "Bucket of Benevolence" (The cylinder) in a simulator. The challenge is to avoid the walls and obstacles present in the Gauntlet. See the picture below for what the Gauntlet looks like:
INSERT GAUNTLET PIC

__The High Level Plan:__
* Use the robot's LIDAR
* Parse Lidar to detect features
    * Use RANSAC algorithm to find circles and lines/walls
* Build a potential field with sources around lines and sinks around the BOB(Bucket of Benevolence)
* Calculate Gradient of this potential field
* Calculate gradeint at robot location and drive accordingly


## Results
Please check out the video here:

[![View Video](http://img.youtube.com/vi/B3F49UcYbRM/0.jpg)](http://www.youtube.com/watch?v=B3F49UcYbRM "Navigating the Gauntlet")

You can also see a plot of the desired and actual path as well as the detected gauntlet and generated gradient (used for gradient descent)
INSERT PLOT
3d Graient pic!

This final plot was generated with encoder data recorded from the robot. It also appears that the robot took ~25 seconds to get to the BOB. This was a long time, the driving time is certainly less. This is due to the fact that it is scanning between each drive step and re-calculating the topography etc. This also explains why my actual plot is slightly different than the calculated plot that was made off of a single scan at the starting point. If navigating with a set path rather than re-scanning than implementing a parametric curve fit to the set points would have allowed for an implemantation of drive control that could go faster rather than driving small segments of straight lines INSERT BOD LINK


## Implementation Strategy and Code
For the code: INSERT LINK

*Please excuse this code, it is made to be able to retrieve scans in the globabl frame to plot things nicely, and parameters must be changed to drive in time steps with relative LIDAR scanning. Plotting the final path must be uncommented and ran with global frame settings with driving disabled*

