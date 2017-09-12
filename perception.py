import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, detect_mode="ground", rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    if detect_mode == "ground":
        above_thresh = ((img[:,:,0] > rgb_thresh[0]) & 
        (img[:,:,1] > rgb_thresh[1]) & (img[:,:,2] > rgb_thresh[2]))
    elif detect_mode == "obstacle":
        above_thresh = ((img[:,:,0] <= rgb_thresh[0]) & 
        (img[:,:,1] <= rgb_thresh[1]) & (img[:,:,2] <= rgb_thresh[2]))
    elif detect_mode == "rock":
        rgb_thresh = ((130, 200), (110, 180), (0, 70))
        above_thresh = (
                        ((img[:,:,0] >= rgb_thresh[0][0]) & 
                            (img[:,:,1] >= rgb_thresh[1][0]) & 
                        (img[:,:,2] >= rgb_thresh[2][0])) & 
                        ((img[:,:,0] <= rgb_thresh[0][1]) & 
                            (img[:,:,1] <= rgb_thresh[1][1]) & 
                        (img[:,:,2] <= rgb_thresh[2][1]))
                       )
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position
    # being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel

# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, 
                                         xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

def world_to_pix(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, -yaw)
    xpix_rot *= scale
    ypix_rot *= scale
    # Apply translation
    # The translation must be rotated to rover frame too
    xpos_rot, ypos_rot = rotate_pix(xpos, ypos, -yaw)
    xpos_rot *= scale
    ypos_rot *= scale
    x_pix_rover, y_pix_rover = translate_pix(xpix_rot, ypix_rot, -xpos_rot, 
                                             -ypos_rot, scale=1)
    # Return the result
    return x_pix_rover, y_pix_rover

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    # keep same size as input image
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, 
                               Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] 
                   - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] 
                   - 2*dst_size - bottom_offset], 
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] 
                   - 2*dst_size - bottom_offset]
                  ])
    # The perspective transform is valid only it the pitch and 
    # roll angles are below this threshold
    pitch_roll_thresh = 1 

    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)
    
    # 3) Apply color threshold to identify navigable 
    # terrain/obstacles/rock samples
    terrain_thresh = color_thresh(warped, detect_mode="ground", 
                                  rgb_thresh=(160, 160, 160))
    obstacle_thresh = color_thresh(warped, detect_mode="obstacle", 
                                   rgb_thresh=(120, 120, 120))
    rock_thresh = color_thresh(warped, detect_mode="rock")
    Rover.rock_thresh = rock_thresh

    # 4) Update Rover.vision_image
    Rover.vision_image[:,:,0] = obstacle_thresh * 255
    Rover.vision_image[:,:,1] = rock_thresh * 255
    Rover.vision_image[:,:,2] = terrain_thresh * 255
    Rover.vision_image = cv2.addWeighted(warped.astype(np.uint8), 1, 
                                Rover.vision_image.astype(np.uint8), 0.5, 0)
        
    # 5) Convert map image pixel values to rover-centric coords
    terrain_x, terrain_y = rover_coords(terrain_thresh)
    obstacle_x, obstacle_y = rover_coords(obstacle_thresh)
    rock_x, rock_y = rover_coords(rock_thresh)
    Rover.rock_x, Rover.rock_y = rock_x, rock_y

    Rover.nav_dists, Rover.nav_angles = to_polar_coords(terrain_x, terrain_y)
    
    # 6) Convert rover-centric pixel values to world coordinates
    xpos, ypos = Rover.pos
    world_size = Rover.worldmap.shape[0]
    scale = 10
    ter_x_world, ter_y_world = pix_to_world(terrain_x, terrain_y, xpos, ypos, 
                                            Rover.yaw, world_size, scale)
    obs_x_world, obs_y_world = pix_to_world(obstacle_x, obstacle_y, xpos, 
                                        ypos, Rover.yaw, world_size, scale)
    rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, xpos, ypos, 
                                              Rover.yaw, world_size, scale)
    if len(rock_x_world) > 0:
        Rover.rock_x_world, Rover.rock_y_world = rock_x_world, rock_y_world
    
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    # Update the worldmap only if the rover's 
    # pitch and roll are below the threshold
    # Convert pitch and roll to be between 0 to 180
    pitch = min(Rover.pitch, np.abs(Rover.pitch - 360)) 
    roll = min(Rover.roll, np.abs(Rover.roll - 360))
    if ((pitch < pitch_roll_thresh) & (roll < pitch_roll_thresh)):
        Rover.worldmap[ter_y_world, ter_x_world, 2] += 1
        Rover.worldmap[obs_y_world, obs_x_world, 0] += 1
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1

    return Rover





