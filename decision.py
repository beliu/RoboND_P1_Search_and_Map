import numpy as np
from perception import *
import time

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    Rover.throttle_set = 0.2
    Rover.stop_forward = 200
    Rover.go_forward = 500
    Rover.brake_set = 4
    xpos, ypos = Rover.pos
    yaw = Rover.yaw
    world_size = Rover.worldmap.shape[0]
    scale = 10
    angle_rec_len = 3
    angle_rec_ix = 0
    travel_time = 5 # How many seconds to pass before sampling the rover's position
    steering_scale = 0.7

    if Rover.nav_angles is not None:

        # Check for Rover.mode status
        if Rover.mode == 'forward': 

            # # Check if the rover is not moving to new terrain, i.e. moving in circles
            # if Rover.prev_pos is None:
            #     Rover.prev_pos = Rover.pos
            # elif ( (np.round(Rover.total_time, 0) > 0) & (Rover.mode != 'find new path') ):
            #     if (np.round(Rover.total_time, 0) % travel_time) == 0:
            #         Rover.travel_dist = np.sqrt((Rover.pos[0] - Rover.prev_pos[0])**2 + (Rover.pos[1] - Rover.prev_pos[1])**2)
            #         Rover.prev_pos = Rover.pos
            #         if Rover.travel_dist < 0.2:
            #             # Reset the stored steering angles
            #             Rover.steer_rec = []
            #             angle_rec_ix = 0
            #             # Let the rover move in a random direction for a second
            #             Rover.t0 = time.time() - Rover.start_time
            #             Rover.throttle = 0.2
            #             Rover.brake = 0
            #             Rover.steer = np.random.randint(-15, 15)
            #             Rover.mode = 'find new path'

            # Check the extent of navigable terrain and make sure there is enough space to move in front of the rover
            if ((len(Rover.nav_angles[np.abs(Rover.nav_angles * 180/np.pi) < 5]) >= 20) & 
                (len(Rover.nav_angles) >= Rover.stop_forward)):  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                Rover.brake = 0
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi) * steering_scale, -15, 15)
            else:
                Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # Reset the stored steering angles
            Rover.steer_rec = []
            angle_rec_ix = 0

            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.1:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            elif Rover.vel <= 0.1:
                if ((len(Rover.nav_angles) < Rover.go_forward) | 
                    (len(Rover.nav_angles[np.abs(Rover.nav_angles * 180/np.pi) < 5]) < 20)):
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 

                # There appears to be a path forward
                else:
                    # If we're stopped but there is a wall to the rover's left then follow the the wall
                    # Otherwise just move forward
                    if np.mean(Rover.nav_angles) < 0:    
                        Rover.mode = 'follow left wall'
                    else:
                        Rover.mode = 'forward'

        elif Rover.mode == "find new path":
            if (Rover.total_time - Rover.t0) >= 1:
                Rover.mode = 'forward'

        elif Rover.mode == 'follow left wall':
            Rover.brake = 0
            if Rover.vel < Rover.max_vel:
                # Set throttle value to throttle setting
                Rover.throttle = Rover.throttle_set
            else: # Else coast
                Rover.throttle = 0

            # Check if the rover needs to stop near an obstacle
            if ((len(Rover.nav_angles[np.abs(Rover.nav_angles * 180/np.pi) < 5]) < 20) | (len(Rover.nav_angles) < Rover.stop_forward)):
                Rover.mode = 'stop'
            else:
                Rover.bias = 1 * len(Rover.nav_angles[Rover.nav_angles > 0]) / 500
                Rover.pre_steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi) * steering_scale, -15, 15)
                Rover.steer = Rover.pre_steer + Rover.bias


    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    
    return Rover
