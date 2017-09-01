## Project: Search and Sample Return
---
[//]: # (Image References)

[image1]: ./output/process_image_output.png
[image2]: ./output/rock_example.jpg
[image3]: ./output/direction_to_move.png

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

First, I updated the `color_thresh()` function in this script from lines 6 to 27. I added the extra parameter for detection that I explained previously in the Training portion. From there, I set fixed points in front of the rover's field-of-vision and fixed points in a top-down image to feed into the perspective transform (see line 93 to 104).
Then, I apply `color_thresh()` three times, one for finding ground, one for obstacles, and one for rocks. For ground, I chose the threshold values `(160, 160, 160)` so that the pixel values need to be above these values to be considered ground. For obstacles, I chose the threshold values `(140, 140, 140)` so that the pixel vlaues need to be below these values. The treshold values for rocks are hard-coded into `color_thresh()` and the user does not need to supply them. After this, I create a 3-channel image where the obstacles are the red layer, the rocks are the green, and the ground are the blue. I then combine this color-threshold image with the warped image so that we can see which portions of the map are detected as obstacle, rock, or ground. All these steps are done in lines 106 to 115.

Next, I find the coordinates of the pixels in the rover's coordinate frame using the function `rover_coords()`, which takes the threshold images as input. I convert these rover-centric coordinates to polar coordinates using the `to_polar_coordinates()` function, which returns an array of the distance to each coordinate from the rover center, and the angle from the x-axis to those coordinates. These two functions are run in lines 118 to 123. 

I convert the rover-centric coordinates to the world coordinate frame using the function `pix_to_world()` and pass in the rover position and yaw from the rover's telemetry. After, I check to see if the rover's pitch and roll angles are below a threshold of 1 degrees. If so, then I update the worldmap with the output of `pix_to_world()`. Otherwise, I leave the worldmap unchanged for the current frame. These functions are run in lines 126 to 140.

Finally, the polar coordinates calculated in line 123 are returned by `perception_step()`.

**decision_step()**

In the `decision_step()` function in the script `decision.py`, I took the output of `perception_step()` and I created four modes that the rover can enter. These modes are:
* Forward
* Stop
* Stuck
* Follow Left Wall
The rover starts out in `forward` mode. In this mode, the rover is using its throttle to move forward and decides which direction to steer by calculating the average of the angles that are output from the `perception_step()`. The image below is an example of taking the rover's camera image and converting it to pixels in the rover's coordinate frame. The red arrow shows the average of the angles that result from converting the rover-centric pixels to polar coordinates.
![Making a Steering Decision][image3]

In order to smooth out the Rover's steering, I take a moving average of the steering angle the script calculates. I take the 5 most recent steering angles and average them. This average steering angle is applied to the Rover via `Rover.steer`.

Of course, as the rover moves it will detect obstacles in its way. The script uses the number of polar coordinates output from `perception_step()` as a proxy for obstacle sensing. So, if there is an obstacle in a certain direction, then the number of polar coordinates in that direction will be low. Using this method, I detect for obstacles in front of the rover by first checking if the total number of coordinates is lower than a threshold. This is implemented by the code `(len(Rover.nav_angles) < Rover.stop_forward))`. Next, there might be cases where there are oepn spaces to both the left and right of the rover, but not directly in front of it. In these cases, the number of polar coordinates that are -5 to 5 degrees in front of the rover will be small. I implement this by the code `((len(Rover.nav_angles[np.abs(Rover.nav_angles * 180/np.pi) < 5]) < 20)`. If both these are not true, then the rover is clear to move forward. Otherwise, if either condition is true, then the rover enters stop mode.

In stop mode, the script first brings the rover to a stop by applying the brakes and turning off the throttle. It then turns the rover -15 degrees (turns to the right) until it senses there are no obstacles ahead. Again, the proxy for obstacle detection is `len(Rover.nav_angles)`. However, I use a higher threshold for when the rover is in stop mode. In this case, the script checks that `(len(Rover.nav_angles) > Rover.go_forward)` and that `(len(Rover.nav_angles[np.abs(Rover.nav_angles * 180/np.pi) < 5]) > 20))`. Once the rover detects a way forward again, it enters `follow left wall` mode.

In `follow left wall` mode, the rover tries to navigate parallel to a wall on its left. Normally, the rover will steer in the direction of the most open space. In order to get it to hug a wall, I calculate the number of pixels that are to the left of the rover by using `len(Rover.nav_angles[Rover.nav_angles > 0]`, scale it, and then add that as a bias term to the steering angle calculated by `np.mean(Rover.nav_angles * 180/np.pi)`. This way, if the rover gets too far from the wall, then the bias term gets larger and pushes the overall steering angle to be positive, which turns the rover to its left towards the wall. If the rover gets too close to the wall, then the bias term is small or even zero and the term `np.mean(Rover.nav_angles * 180/np.pi) + bias` is negative, turning the rover to its right away from the wall. Again, at any time during its movement, if the rover encounters an obstacle, it will go into stop mode and turn in place until it finds a new path forward.

Lastly, I implemented `stuck` mode because I saw that sometimes the rover would run over small rocks and not move. For each frame, I check if the rover's throttle is on but its speed is close to zero or it there is a steering angle applied but the rover yaw is not changing. If this is the case, I first record the time that this occurred but otherwise change nothing. I wait for three seconds to lapse before I check for any movement again. If the rover still has not turned or moved, then it entrs `stuck` mode. In this mode, the rover simply backs up and turns in the direction of most open space. Once there is enough navigable terrain detected ahead, the rover enters `forward` mode.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**



