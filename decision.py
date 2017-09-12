import numpy as np
from perception import *
import time

# This is where you can build a decision tree for determining throttle,     
# brake and steer commands based on the output of the perception_step()     
# function
def decision_step(Rover):
    # Getting some key data
    xpos, ypos = Rover.pos
    yaw = Rover.yaw
    world_size = Rover.worldmap.shape[0]
    scale = 10 
    rec_time = 5 # The number of seconds that before sampling 
                 # the rover's position

    ## Doing some preliminary checks of the rover's  
    ## movements and surroundings
    # Check if there are any rock samples in the rover's vision
    # if Rover.mode != 'collect sample':
    if ((not Rover.picking_up) & (Rover.mode != 'stuck')):
        if ((Rover.rock_thresh.any()) | Rover.near_sample):
            Rover.mode = 'collect sample'
    if Rover.picking_up:
        Rover.mode = 'picking up'
    # Check if the rover is stuck
    elif not Rover.check_stuck:
        if ((Rover.throttle != 0) & (Rover.vel < 0.001)):
            # Get a record of the rover's state for reference to later
            Rover.t0 = Rover.total_time
            Rover.prev_pos = Rover.pos
            Rover.prev_yaw = yaw
            Rover.check_stuck = True
    else:
        # After a certain number of seconds have elpased, check if 
        # the rover has moved or turned
        if (Rover.total_time - Rover.t0) >= rec_time:
            if Rover.mode != 'stuck':
                Rover.dist_traveled = np.sqrt((Rover.pos[0] - 
                                               Rover.prev_pos[0])**2 + 
                                               (Rover.pos[1] - 
                                                Rover.prev_pos[1])**2)
                not_turning = ((Rover.mode == 'stop') & (Rover.steer != 0) & 
                                (abs(Rover.yaw - Rover.prev_yaw) < 5))
                not_moving = ((Rover.throttle != 0) & (not Rover.picking_up) & 
                              (Rover.dist_traveled < 0.2))
                # If the rover has not moved or turned even though it 
                # should have, enter stuck mode and get a record of the
                # rover's current state
                if (not_turning | not_moving):  
                        Rover.prev_mode = Rover.mode      
                        Rover.mode = 'stuck'
                        Rover.t0 = Rover.total_time
                        Rover.prev_yaw = yaw
                        Rover.prev_pos = Rover.pos
                else:
                    Rover.check_stuck = False
        
    if ((len(Rover.nav_angles) > 0) | Rover.picking_up | Rover.near_sample):
        # Calculate some sensory data in front of the rover's field of vision
        # Consider the data within 3 meters in front of the rover
        front_angles = Rover.nav_angles[(Rover.nav_dists / 10) <= 3] 
        Rover.front_angles = front_angles
        # Consider data within +/- 5 degrees in front of the rover
        Rover.front_angles_ahead = front_angles[(np.abs(front_angles * 
                                                180/np.pi)) <= 5]
        Rover.left_angles = front_angles[front_angles * 180/np.pi > 5]
        Rover.right_angles = front_angles[front_angles * 180/np.pi < -5]

        if Rover.mode == 'forward': 
            Rover.steer = 0
            if Rover.vel < Rover.max_vel:
                # Set throttle value to throttle setting
                Rover.brake = 0
                Rover.throttle = Rover.throttle_set
            else: # Else coast
                Rover.throttle = 0
            # Stop when there are obstacles in front
            if ((len(Rover.front_angles) < 75) | 
                (len(Rover.front_angles_ahead) < 30)):
                Rover.mode = 'stop'
                # Turn the rover in the direction of the most
                # navigable terrain
                left_angles = Rover.nav_angles[Rover.nav_angles > 0]
                right_angles = Rover.nav_angles[Rover.nav_angles < 0]
                Rover.turn_direction = (len(left_angles) - len(right_angles))
            else:
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                Rover.brake = 0
                steering_scale = 1
                bias = 0

                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                # If the rover starts sensing an obstacle ahead,
                # try to do a sharp turn away from it
                if len(Rover.front_angles_ahead) < 60:
                    if np.sign(np.mean(Rover.nav_angles)):
                        # Add this bias to steering to maneuver the
                        # rover away from the obstacle
                        bias = np.sign(np.mean(Rover.nav_angles)) * 15
                        Rover.throttle = 0.4
                steer = np.clip(np.mean(Rover.front_angles * 180/np.pi) * 
                                steering_scale + bias, -15, 15)
                Rover.steer = steer
     
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            Rover.throttle = 0
            if Rover.vel > 0.2:
                Rover.brake = Rover.brake_set
            else:
                Rover.brake = 0
            # Turn in the direction with the most navigable terrain
            if Rover.turn_direction > 0:
                Rover.steer = 15
            else:
                Rover.steer = -15

            # There appears to be a path forward
            if ((len(Rover.nav_angles) >= 1000) & 
                (len(Rover.front_angles_ahead) >= 70)):
                # If we're stopped but there is a wall to the 
                # rover's left then follow the the wall
                # Otherwise just move forward
                if len(Rover.left_angles) < 200:    
                    Rover.mode = 'follow left wall'
                else:
                    Rover.mode = 'forward'   
                Rover.prev_mode = 'stop'    

        # In this mode, the rover attempts to travel along a wall to 
        # its left at a constant distance
        elif Rover.mode == 'follow left wall':
            Rover.brake = 0
            if Rover.vel < Rover.max_vel:
                # Set throttle value to throttle setting
                Rover.throttle = Rover.throttle_set
            else: # Else coast
                Rover.throttle = 0

            # Check if the rover needs to stop near an obstacle
            if ((len(Rover.front_angles) < 75) | 
                (len(Rover.front_angles_ahead) < 10)):
                Rover.mode = 'stop'
                Rover.turn_direction = (len(Rover.left_angles) - 
                                        len(Rover.right_angles))
            # If we pull too far away from the wall
            elif len(Rover.left_angles) > 300:
                Rover.mode = 'forward'
            else:
                l = 100 # The closest we want to get 
                        # to the wall before turning away
                r = 130 # The farthest we want to get
                        # away from the wall before turning to it
                m = 15 / l # Slope of steering angle change
                b = -r * m
                steering_scale = 0.7

                # If we are too close to the wall, turn to the right
                if len(Rover.left_angles) < l:                
                    steer = np.clip((-15 + m * len(Rover.left_angles)) * 
                                    steering_scale, -15, 0)
                # If we are too far from the wall, turn to the left
                elif len(Rover.left_angles) > r:
                    steer = np.clip((b + m * len(Rover.left_angles)) * 
                                    steering_scale, 0, 15)                
                # Try to maintain this distance from the wall
                else:
                    steering_scale = 0.2
                    # This bias term grows as the rover 
                    # gets farther from the wall
                    Rover.bias = len(Rover.left_angles) / 100 
                    # The larger the bias term, the more the rover 
                    # will steer to the left
                    steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi) * 
                                    steering_scale + Rover.bias, -15, 15)
                Rover.steer = steer

        # In this mode, the rover is not moving even though it should be
        elif Rover.mode == 'stuck':
            # Check the surroundings to the rover's immediate 
            # left and right side which is usually where the 
            # rover will get snagged
            front_angles = Rover.nav_angles[(Rover.nav_dists / 10) <= 1]
            left_angles = front_angles[front_angles * 180/np.pi > 10]
            right_angles = front_angles[front_angles * 180/np.pi < -10]

            # Here we will cycle through a number of different
            # methods to get the rover unstuck
            if Rover.stuck_type is None:  
                if (len(Rover.front_angles) >= 70): 
                    if len(left_angles) < 20:
                        Rover.stuck_type = 'avoid left wall'
                    elif len(right_angles) < 20:
                        Rover.stuck_type = 'avoid right wall'
                    elif ((len(left_angles) < 20) & (len(right_angles) < 20)):
                        Rover.stuck_type = 'back up'
                else:
                    Rover.stuck_type = 'turn to open'
                    Rover.prev_yaw = yaw
                Rover.t0 = Rover.total_time
                Rover.brake = 0
            # Turn toward an opening
            elif Rover.stuck_type == 'turn to open':      
                Rover.throttle = 0      
                Rover.steer = Rover.stuck_steer
            # Try to move forward
            elif Rover.stuck_type == 'break free':
                Rover.steer = 0 
                Rover.throttle_set = 0.2
                Rover.throttle = Rover.throttle_set
            # Move in reverse
            elif Rover.stuck_type == 'back up':
                Rover.throttle = -0.2
                Rover.steer = 0
            # Rover is snagged on left, try to back up and turn away
            elif Rover.stuck_type == 'avoid left wall':
                Rover.steer = -Rover.stuck_steer
                Rover.throttle = -Rover.throttle_set
            # Rover is snagged on right, try to back up and turn away
            elif Rover.stuck_type == 'avoid right wall':
                Rover.steer = -Rover.stuck_steer
                Rover.throttle = Rover.throttle_set

            # Check if rover has moved in elapsed time    
            if (Rover.total_time - Rover.t0) >= 4:
                Rover.dist_traveled = np.sqrt((Rover.pos[0] - 
                    Rover.prev_pos[0])**2 + (Rover.pos[1] - 
                    Rover.prev_pos[1])**2)
                # The rover has not moved much at all so try a 
                # different way to get unstuck
                if Rover.dist_traveled < 0.03:
                    if ((Rover.stuck_type == 'avoid left wall') | 
                        (Rover.stuck_type == 'avoid right wall')):
                        Rover.stuck_type = 'turn to open'
                    elif Rover.stuck_type == 'break free':
                        Rover.stuck_type = 'turn to open'
                    elif Rover.stuck_type == 'back up':
                        Rover.stuck_type = None
                    elif Rover.stuck_type == 'turn to open':
                        # The rover hasn't been able to turn either
                        if (abs(yaw - Rover.prev_yaw)) < 45:
                            Rover.stuck_type = 'back up'
                        else:
                            Rover.stuck_type = 'break free'
                # Update the refernce time
                Rover.t0 = Rover.total_time    
            # The rover has been able to cover some distance
            if Rover.dist_traveled >= 0.1:
                Rover.stuck_type = None
                Rover.throttle = Rover.throttle_set
                Rover.mode = Rover.prev_mode
                Rover.t0 = Rover.total_time
                Rover.check_stuck = False

        elif Rover.mode == 'collect sample':
            steering_scale = 1
            # If the rock sample is now being detected by the rover
            if (len(Rover.rock_x) > 0):
                # Move toward the rock coordinates
                Rover.nav_dists, Rover.nav_angles = (
                                  to_polar_coords(Rover.rock_x, Rover.rock_y))
                steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), 
                                -15, 15) * steering_scale
            # Lost the detection but we have the samples saved
            # world position
            else:
                # Convert from world to rover frame
                rock_x, rock_y = world_to_pix(Rover.rock_x_world, 
                    Rover.rock_y_world, xpos, ypos, yaw, world_size, scale)
                Rover.nav_dists, Rover.nav_angles = (
                                            to_polar_coords(rock_x, rock_y))
                steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), 
                                -15, 15) * steering_scale
                Rover.world_to_pix_rock_x = rock_x
                Rover.world_to_pix_rock_y = rock_y
            Rover.rock_dist = np.mean(Rover.nav_dists / 10)

            if (not Rover.near_sample):
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.brake = 0
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.steer = steer
            else:  
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
    
        elif Rover.mode == 'picking up':
            if not(Rover.picking_up):
                if (((len(Rover.front_angles)) < 20) | 
                    (len(Rover.front_angles_ahead) < 5)):
                    Rover.mode = 'stop'
                    Rover.turn_direction = (len(Rover.left_angles) - 
                                            len(Rover.right_angles))
                 # There appears to be a path forward
                else:
                    # If we're stopped but there is a wall to 
                    # the rover's left then follow the the wall
                    # Otherwise just move forward
                    if len(Rover.left_angles) < 30:    
                        Rover.mode = 'follow left wall'
                    else:
                        Rover.mode = 'forward'  

    else:
        Rover.mode = 'stop'
        Rover.brake = 0
        Rover.steer = -15

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover
