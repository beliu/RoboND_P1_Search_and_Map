## Project: Search and Sample Return
---
[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg 


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
#### 1. I will cover how I filled in the functions that are present in the Notebook
** Color Threshold **
Within `color_thresh()`, I added an extra parameter called `detect_mode` in line 3 of that cell. I use this parameter to choose whether I am interested in detecting obstacles, navigable terrain, or rocks. For example, if I wanted to detect obstacles, I would set `detect_mode = 'obstacle'`. 
Lines 9 through 20 will choose one of three image channels to apply threshold values to based on `detect_mode`. For obstacles and terrain, the threshold values for each image channel is supplied by the user. For rocks, I plotted an interactive image of a rock in the notebook and found a range of values that the yellow rock can take on across all three channels. I then hard-coded those values as threshold values into `color_thresh()` for `detect_mode = 'rock'` only.
Line 22 then takes the pixels that meeth the threshold values and sets them to 1 in the output threshold image.





---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.


![alt text][image1]

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 
And another! 

![alt text][image2]
### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.


#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  



![alt text][image3]


