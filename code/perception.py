import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,:])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met

    # Index the array of zeros with the boolean array and set to 1
    path = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])  
    
    color_select[path,2] = 255
    
    rock = (img[:,:,0] > 140) \
                & (img[:,:,1] > 110) \
                & (img[:,:,2] < 100)
    # Index the array of zeros with the boolean array and set to 1
    color_select[rock,1] = 255
    
    stone = (img[:,:,0] < 160) \
             & (img[:,:,0] >1) \
             & (img[:,:,1] < 160) \
             & (img[:,:,1] > 1) \
             & (img[:,:,2] < 160) \
             & (img[:,:,2] > 1)  
            
    # Index the array of zeros with the boolean array and set to 1
    color_select[stone,0] = 255
    
    
    
    return color_select


def color_ruda(img):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    ruda = (img[:,:,0] > 140) \
                & (img[:,:,1] > 110) \
                & (img[:,:,2] < 100)
    # Index the array of zeros with the boolean array and set to 1
    color_select[ruda] = 1

    return color_select



# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
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

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # TODO:
    # Convert yaw to radians
    # Apply a rotation
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # TODO:
    # Apply a scaling and a translation
    xpix_translated = np.int_(xpos + (xpix_rot / scale))
    ypix_translated = np.int_(ypos + (ypix_rot / scale))
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])

    warped = perspect_transform(Rover.img, source, destination)
    threshed = color_thresh(warped)
    Rover.vision_image[:,:,0] = threshed[:,:,0]
    Rover.vision_image[:,:,1] = threshed[:,:,1]
    Rover.vision_image[:,:,2] = threshed[:,:,2]
    
    xpix_rock, ypix_rock = rover_coords(threshed[:,:,0])
    xpix_stone, ypix_stone = rover_coords(threshed[:,:,1])
    xpix_path, ypix_path = rover_coords(threshed[:,:,2])
    
    xpix_rock_world, ypix_rock_world = pix_to_world(xpix_rock, ypix_rock, Rover.pos[0],                       Rover.pos[1],Rover.yaw, Rover.worldmap.shape[0], 10)
    xpix_stone_world, ypix_stone_world = pix_to_world(xpix_stone, ypix_stone, Rover.pos[0],                       Rover.pos[1],Rover.yaw, Rover.worldmap.shape[0], 10)
    xpix_path_world, ypix_path_world = pix_to_world(xpix_path, ypix_path, Rover.pos[0],                       Rover.pos[1],Rover.yaw, Rover.worldmap.shape[0], 10)
    
    #Update Rover worldmap (to be displayed on right side of screen)
    Rover.worldmap[ypix_rock_world, xpix_rock_world, 0] += 1
    Rover.worldmap[ypix_stone_world, xpix_stone_world, 1] += 1
    Rover.worldmap[ypix_path_world, xpix_path_world, 2] += 1

 
    # Update Rover pixel distances and angles
    dist, angles = to_polar_coords(xpix_path, ypix_path)
    Rover.nav_dists = dist
    Rover.nav_angles = angles
  

    
    
    return Rover