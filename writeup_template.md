## Project: Search and Sample Return
---
[//]: # (Image References)

[image1]: ./output/process_image_output.png
[image2]: ./output/rock_example.jpg
[image3]: ./output/direction_to_move.png
[image4]: ./output/transform_inverse.jpg

### The goals / steps of this project are the following:
**Training / Calibration** 
* Download the rover simulator and record training images on which I can test the `process_image()` function.
* Add functions to detect obstacles and rocks.
* Fill in `process_image()` so that it can perform the following :
  * take training images, warp the perspective to be a top-down view of the rover's field of vision, 
  * apply color thresholds to discern navigable terrain, obstacles, and rocks,
  * convert the rover's perspective to be in the world coordinate system,
  * find the obstacle and terrain areas in the world coordinate system and overlay them on a ground-truth map.
* Use `moviepy` to create a video that shows the output of `process_image()`.

**Autonomous Navigation / Mapping**
* Fill in the `perception_step()` function within the `perception.py` script with similar image-processing functions in `process_image()`.
* Update Rover data with the output of `perception_step()` so that the data can be used to help the rover autonomously navigate the map.
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until the rover can to the following:
 * find and move toward navigable terrain,
 * detect and avoid running into obstacles,
 * follow the mountain walls to find new pathways,
 * get out of situations in which it may become stuck,
 * mark rocks as located when it comes across them,
 * cover at least 40% of the map without getting stuck,
 * achieve a minimum of 60% fidelity in the mapping.

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**
I used a screen resolution of 1024 x 768, a graphics quality setting of GOOD, and got the FPS ranged from 31 to 33 on all of my runs. 

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.
### Notebook Analysis
#### 1. I will cover how I filled in the key functions that are present in the Notebook
**Color Threshold**

Within `color_thresh()`, I added an extra parameter called `detect_mode` in line 3 of that cell. I use this parameter to choose whether I am interested in detecting obstacles, navigable terrain, or rocks. For example, if I wanted to detect obstacles, I would set `detect_mode = 'obstacle'`. 
Lines 9 through 20 will choose one of three image channels to apply threshold values to based on `detect_mode`. For obstacles and terrain, the threshold values for each image channel is supplied by the user. For rocks, I plotted an interactive image of a rock in the notebook and found a range of values that the yellow rock can take on across all three channels. For example, see below for a plot of a rock sample and its corresponding pixel values in the lower left corner:
![Rock sample pixel values][image2]
I then hard-coded those values as threshold values into `color_thresh()` for `detect_mode = 'rock'` only.
Line 22 then takes the pixels that meeth the threshold values and sets them to 1 in the output threshold image.

**Databucket Class**

The CSV file sometimes recorded negative pitch and roll angles. However, the angles should be in the range 0 to 360. Therefore I set each negative angle to 0. Also, I converted the pitch and roll angles to be between -180 and 180. I did this because I set thresholds for the pitch and roll angles for mapping the terrain. If those angles exceeded the threshold, then the perspective transform would not have produced a valid image. It was easier to compare angles to these thresholds if I converted them to be between -180 and 180 versus 0 and 360. For example, if I set the pitch threshold to be 2 degrees, then the valid angles would be 0 to 2 or 358 to 360. If we convert 358 -> 360 to -2 -> 0, then it's easier to just take the absolute value of the pitch and compare that to the threshold of 2. 

**calc_perct_mapped**

I added a function that takes in rover data and calculates the percentage of the worldmap that the rover has already visited. This script was taken primarily from lines 72 - 88 and lines 115 - 123 of the `supporting_functions.py` script.

**process_image**

This function takes an image from the training data. I apply a perspective transform to the image and apply color threshold to detect the obstacles, the ground, and rocks from the transformed image. Then, I create a 3-channel image by stacking these threshold images on top of each other. This 3-channel image is called `thresh_3d` and it is created in line 47 of this cell. This image is added to the warped image so you can see which parts of the transformed image is classifed as obstacle, ground, or rock. 

Using the individual threshold images, I use `rover_coords()` to find the pixels associated with the thresholds and then convert them to the world coordinate frame using `pix_to_world`. `pix_to_world()` requires the rover coordinates, the x and y positions of the rover, the rover's yaw w.r.t. the world coordinate frame's x-axis, and size of the world map, and scale. 

If the pitch and roll angles are below a threshold of 1 degrees, then we can say the perspective transform is valid and we can update the worldmap with the output of the `pix_to_world()` function. For each layer, we take the respective coordinates and increment the pixel value at those coordinates. However, because the rover is constantly moving, certain areas that were previously identified as ground may become identified as an obstacle once the rover changes its field of vision. In this case, I don't want to make the obstacle pixel value too strong in a location that is also marked as ground. Therefore, I check if a location is classified as ground with high-confidence. By this, I mean this location has been classified as ground in many frames. Such an area will have a high pixel value in the ground layer. If the pixel value is over a threshold (I chose 80 for my script), then I set the pixel values for this area in the obstacle layer to 0, meaning that there should be no obstacles in this area. The lines in the script that do this are located 71 to 78 in this cell.

The output image combines the various images described above. The normal image is in the upper left, the warped image is the upper right, the world mapping to ground truth is in the lower left, and the warped plus threshold image is in the lower right. The following is an example of an image output from `process_image()`:

![Image Output of process_image()][image1]

### Autonomous Navigation and Mapping
#### In the following section, I describe how to take Rover telemetry to guide the Rover to autonomously navigate the world map. The scripts below take Rover visual input and telemetry to udpate its internal state and allow it to make decisions on how to travel the map. The Rover's internal state itself is implemented as a Python Class that is updated by the functions I explain below.
#### 1. I will describe the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts.
**perception_step**

The function `perception_step()` uses a number of helper functions to process the image data the rover records. I will describe the most important ones below:

First, I updated the `color_thresh()` function in this script from lines 6 to 31. I added the extra parameter for detection that I explained previously in the Training portion. From there, I set fixed points in front of the rover's field-of-vision and fixed points in a top-down image to feed into the perspective transform (see line 117 to 126).
Then, I apply `color_thresh()` three times, one for finding ground, one for obstacles, and one for rocks. For ground, I chose the threshold values `(160, 160, 160)` so that the pixel values need to be above these values to be considered ground. For obstacles, I chose the threshold values `(120, 120, 120)` so that the pixel vlaues need to be below these values. The treshold values for rocks are hard-coded into `color_thresh()` and the user does not need to supply them. After this, I create a 3-channel image where the obstacles are the red layer, the rocks are the green, and the ground are the blue. I then combine this color-threshold image with the warped image so that we can see which portions of the map are detected as obstacle, rock, or ground. All these steps are done in lines 144 to 148.

Next, I find the coordinates of the pixels in the rover's coordinate frame using the function `rover_coords()`, which takes the threshold images as input. I convert these rover-centric coordinates to polar coordinates using the `to_polar_coordinates()` function, which returns an array of the distance to each coordinate from the rover center, and the angle from the x-axis to those coordinates. These two functions are run in lines 151 to 156.

I convert the rover-centric coordinates to the world coordinate frame using the function `pix_to_world()` and pass in the rover position and yaw from the rover's telemetry. After, I check to see if the rover's pitch and roll angles are below a threshold of 1 degrees. If so, then I update the worldmap with the output of `pix_to_world()`. Otherwise, I leave the worldmap unchanged for the current frame. These functions are run in lines 158 to 180.

I pay special attention locating rock samples within `perception.py`. If `color_thresh()` detects rock samples in line 140, I convert the rock threshold to be in both the rover's frame and the world frame. In order to accomplish the latter, I added a new function to the script called `world_to_pix` in lines 84 to 97. Converting a set of points from a world frame to the rover frame requires doing the inverse of the `pix_to_world` operations. To do this, I implemented the following transformation that accomplishes a transform inverse:

![Transform Inverse][image4]

(Source: Peter Corke's Robotics Vision and Control, pg 22) R is the rotation operation and t is the translation operation. I will explain the reason why I need rock coordinates in both frames when I describe autonomous navigation.

At the end of the function, all the incoming data has been processed and converted into numeric data that the rover can use to navigate the map. The internal state of the rover is stored in the Rover class and is output by `perception_step()`.

**decision_step()**

In the `decision_step()` function in the script `decision.py`, I took the output of `perception_step()` and use the Rover data to help the robot make decisions about how to navigate the map. To help with this process, I created four primary modes that the rover can enter. These modes are:
* Forward
* Stop
* Stuck
* Follow Left Wall
I also created two modes for collecting samples. One mode is simply `collecting samples` when the Rover detects a sample and the other is `picking up` when the rover is picking the sample up.

##### Sensing the Rover's Surroundings #####
Before I describe the modes in more detail, I would like to explain how I use output data from `perception_step()` to sense the rover's surroundings. The script uses the number of polar coordinates output from `perception_step()` as a proxy for obstacle and sample sensing. So, if there is an obstacle in a certain direction, then the number of polar coordinate data points in that direction will be low. On the other hand, if the number of data points is high in that direction, this indicates there is open terrain there. Using this method, I detect for obstacles in front of the rover by checking if the total number of data points is lower than some threshold. Conversely, if the number of data points is higher than some threshold, than we see an opening. This type of sensing can be implemented using `len(Rover.nav_angles)`. However, there are some nuances I discovered while debugging the navigation.

First, I discovered that while the rover is around boulders, it would often get stuck thinking it could move forward when infact there was a large boulder in front of it. This occurred because the rover's field of vision wraps around the boulder and `decision_step` was counting data points beyond the boulder when deciding if there was a way forward. However, by limiting the field to just what is 3 meters in front of the rover, the boulder will feature prominently in the rover's field of vision and will reduce the number of angles detected, therefore increaseing the chances that the script will decide there is an obstacle ahead. I limit the field of vision to 3 meters using `front_angles = Rover.nav_angles[(Rover.nav_dists / 10) <= 3]` in line 62. From this set of data points, I can derive other information that gives an even finer description of the rover's surroundings. For example, I created a variable called `front_angles_ahead` that only contains data points within a +/- 5 degree range in front of the rover. I use this to detect obstacles directly in front of the rover. Furthermore, I use `left_angles` and `right_angles`, which contain data points more than 5 degrees to the left and right of the rover's field of vision, respectively. These variables are created in lines 63 to 68. I make use of these variables extensively in the script for all my surrounding detection needs.

##### Rover Decision Modes #####
Now, let's move on to the different Rover modes. At the start of `decision_step`, from lines 27 to 33, I check if the rover is stuck. To do this, I detect when the rover's speed is near 0 but it has throttle. Instead of immediately going into `stuck` mode, I wait 5 seconds before checking if the rover has moved or turned at all. If not, then the rover is in `stuck` mode. Before I describe what happens then, I will first explain what happens if the Rover is not stuck.

The rover starts out in `forward` mode. In this mode, the rover is using its throttle to move forward and decides which direction to steer by calculating the average of the angles that are stored in `Rover.nav_angles` which were calculated from the `perception_step()`. The image below is an example of taking the rover's camera image and converting it to pixels in the rover's coordinate frame. The red arrow shows the average of the angles that result from converting the rover-centric pixels to polar coordinates.
![Making a Steering Decision][image3]

Sometimes, there may be an obstacle looming ahead of the rover but the steering angle calculated by `Rover.nav_angles` is not sharp enough. In these cases, I number of `nav_angles` data points directly in front of the rover and if that number drops below a certain threshold, I apply a sharp 15 degree turn in the direction that has the most open terrain (see lines 101 to 108). In order to smooth out the Rover's steering, I noticed that it helps to limit the rover's field of vision to be just 3 meters in front of the rover. This prevents the oscillations that sometimes occur when you consider terrain way out in front of the rover.

If an obstacle is detected and the rover gets sufficiently close to it, then the rover enters `stop` mode and the script first brings the rover to a stop by applying the brakes and turning off the throttle. The rover then turns in the direction where there is more navigable terrain by comparing the number of data points to the left of the rover and to the right. Once the rover senses an opening, it checks if it is close to a wall on its left. If so, it enters `follow left wall` mode and if not, it goes into `forward mode`. `stop` mode is implemented in lines 112 to 135.

In `follow left wall` mode, the rover tries to navigate parallel to a wall on its left. Normally, the rover will steer in the direction of the most open space. In order to get it to hug a wall, I set a limit for how close or far away the rover can get to the wall. If the rover gets closer than the limit, then it will attempt to turn away. If it gets too far from the wall, then it will attempt to turn toward it. However, if the rover gets too far away, i.e. if `len(Rover.left_angles)` is larger than some threshold, then we decide that we lost the wall altogether and go back to `forward` mode. `follow left wall` mode is implemented in lines 139 to 183. 

Next, I implemented `stuck` mode to help get the rover back on track when it gets snagged by a rock or outcropping. The script checks to see if the rover has covered any distance over a certain predefined number of seconds. If not, then the rover will check to see if it's been snagged on its left or right. In either situation, it will attempt to back up and turn away from the side it's been snagged on. If the rover still has not covered a certain distance after a number of seconds, then it will attempt to turn in place. If it can successfully turn, as evidenced by a change in its yaw, then the rover will begin to move again when it detects an opening. Finally, if the rover has covered distance above a threshold I set, then I assume that it has freed itself from being stuck. The rover then returns to the mode that it was in before it got stuck. These functions are implemented in lines 186 to 260.

Finally, we have the two modes that deal with collecting rock samples. Earlier, I mentioned that when the `color_threshold` function from `perception_step` detects a sample, then the script from `perception.py` converts the threshold image into data points in the rover's reference frame. These data points are stored in `Rover.rock_x` and `Rover.rock_y`. The script in `decision_step` converts those data points into polar cooridnates and use the mean angle calculated from the polar coordinates to steer the rover toward the sample. During testing, I noticed that sometimes the sample would not be detected in certain frames even as the rover was heading in the general direction of the sample. This would often cause the rover to switch to another state or to veer off course. As a remedy, I added a function to keep a record of the location of the sample. I could not store the polar coordinates from the rover's reference frame because it keeps moving and the previously saved coordinates would no longer be accurate at a later time. Therefore, I convert the rover coordinates into a world reference frame, which would not change even as the rover moves. If the sample ever drops out of `color_threshold` detection, then the script calls up the saved world coordinates and converts it back to the rover's frame using its current x-y position and yaw. If these coordinates, now in the rover's frame, are converted into polar coordinates, then the angles can be used to guide the rover to the sample. I implement this function in 262 to 283.

Once the rover is near the sample, its own internal telemtry will toggle `Rover.near_sample`. Once that is true, I make the rover stop and it will automatically pickup the sample. Since the rover is not moving during this time and it does take a few seconds to pick up the sample, I don't want the script to think the rover is stuck so I do not check for distances covered during this mode.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  
After implementing the functions above, I ran the rover in autonomous several times to see if it could cover at least 40% of the map at 60% fidelity while being able to pick up samples it sees and free itself if it ever gets stuck. Overall, the rover was able to achieve the minimum requirements. There are a few areas where I could improve. I added several different sensing directions for the rover but it still has trouble navigating around boulders and rocks. Sometimes the number of data points doesn't seem to go down far enough as the rover approaches a boulder, even as I limit its field of vision to be more in front of it. With enough tweaks, I was able to get it to either avoid most boulders or at least free itself it it got stuck but the movements are not smooth. Similarly, the cost of adding all these rules to carefully avoid boulders is that it acts too cautiously around walls. There are times when the rover could keep going and then do a sharp turn to follow the wall but instead it opts to stop completely and turn in place to find an opening. While this still means the rover can navigate to almost every place of the map without getting stuck, it also looks somewhat jerky. Lastly, I could work further on the challenge and direct the rover to pickup every rock while also mapping 100% of the map, although I will save that for another time.





