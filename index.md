# Navigating the Gauntet!
## The Challenge
The Gauntlet challenge is to navigate a robot successfully from a starting position in to the "Bucket of Benevolence" (The cylinder) in a simulator. The challenge is to avoid the walls and obstacles present in the Gauntlet. See the picture below for what the Gauntlet looks like:

![Gauntlet Top View](/gauntlet/pics/gauntletTop.PNG)


__The High Level Plan__ (Level 3)(See code and implentation section for more details)
* Use the robot's Lidar to obtain data about surrounding area
* Parse Lidar to detect features
    * Use RANSAC algorithm to find circles and lines/walls
* Build a potential field with sources around lines and sinks around the BOB(Bucket of Benevolence)
* Calculate Gradient of potential field at robot location and drive accordingly using gradient descent

## Results
Please check out the video here:

[![View Video](http://img.youtube.com/vi/B3F49UcYbRM/0.jpg)](http://www.youtube.com/watch?v=B3F49UcYbRM "Navigating the Gauntlet")

You can also see a plot of the desired and actual path as well as the detected gauntlet 
![Gauntlet Mapping](/gauntlet/pics/gauntletMapping.png)

As well as the generated topography (used to calulate the gradient shown above for gradient descent)
![3d Generated Map](/gauntlet/pics/Gauntlet%20Mesh.png)

This plot shows the original calculations with the actual traveled path overlayed
![3d Generated Map](/gauntlet/pics/actualpath.png)

This final plot was generated with encoder data recorded from the robot. It also appears that the robot took ~25 seconds to get to the BOB. This was a long time, the driving time is certainly less. This is due to the fact that it is scanning between each drive step and re-calculating the topography etc. This also explains why my actual plot is slightly different than the calculated plot that was made off of a single scan at the starting point. If navigating with a set path rather than re-scanning than implementing a parametric curve fit to the set points would have allowed for an implemantation of drive control that could go faster rather than driving small segments of straight lines 


## Implementation Strategy and Code
For the code: [View Code](/gauntlet/gauntletNav.m)
*Please excuse this code, it is made to be able to retrieve scans in the globabl frame to plot things nicely, and parameters must be changed to drive in time steps with relative LIDAR scanning. Plotting the final path must be uncommented and ran with global frame settings with driving disabled*

Broken down from into the main points above:
### Use the robot's Lidar to obtain data about surrounding area

### Parse Lidar to detect features
* How RANSAC Works
* Applications to Cicrlces and Lines
* Problems with RANSAC and how they were addressed
    
### Build a potential field with sources around lines and sinks around the BOB(Bucket of Benevolence)
* Problems wiht my potential field generation

### Calculate Gradient at robot location and drive accordingly using gradient descent


