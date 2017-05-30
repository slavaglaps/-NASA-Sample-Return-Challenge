# Project: Search and Sample Return

<a href="https://youtu.be/AC45vtDonzM" target="_blank"><img src="http://img.youtube.com/vi/AC45vtDonzM/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

## The goals / steps of this project are the following:  

The goal of this project was to teach the robot to autonomously navigate on Mars using the basic methods of computer vision.


## CODE

`drive_rover.py` sctipt with Robot class

`perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data.

`decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands.

`supporting_functions.py` In update_rover() your RoverState() object gets updated with each new batch of telemetry. The create_output_images() function is where your Rover.worldmap is compared with the ground truth map and gets converted, along with Rover.vision_image, into base64 strings to send back to the rover.

## Launch in Autonomous Mode

To get started with autonomous driving in the simulator, go ahead and run drive_rover.py by calling it in the following manner at the terminal prompt (and you should see similar output as displayed below):

`python drive_rover.py
(1439) wsgi starting up on http://0.0.0.0:4567`

You can also record images while in autonomous mode by providing a path to where you want to save images when you call drive_rover.py like this:

```python drive_rover.py path_to_folder```

## Autonomous Navigation / Mapping


For navigation, I used this algorithm

**0) Get the image from the camera**

   ![Alt text](readme_image/image1.png?raw=true "Optional Title")
   
**1)  Perspective Transform**

  ![Alt text](readme_image/image2.png?raw=true "Optional Title")
   
   ```python
   dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    warped = perspect_transform(Rover.img, source, destination)
    ```
**2) Color Thresholding**    

   ![Alt text](readme_image/image3.png?raw=true "Optional Title")

   ```python   
   threshed = color_thresh(warped)
   Rover.vision_image[:,:,0] = threshed[:,:,0]
   Rover.vision_image[:,:,1] = threshed[:,:,1]
   Rover.vision_image[:,:,2] = threshed[:,:,2]
   ```

   
**3)to Polat Coordinate**  

   ![Alt text](/readme_image/image4.png?raw=true "Optional Title")

   ```python 
   xpix_rock, ypix_rock = rover_coords(threshed[:,:,0])
   xpix_stone, ypix_stone = rover_coords(threshed[:,:,1])
   xpix_path, ypix_path = rover_coords(threshed[:,:,2])
   ```
    
**4)From Rover coordinate frame to World coordinate**      

   ![Alt text](/readme_image/image5.png?raw=true "Optional Title")
   ```python 
   xpix_rock_world, ypix_rock_world = pix_to_world(xpix_rock, ypix_rock,Rover.pos[0], 
                                                     Rover.pos[1],Rover.yaw,Rover.worldmap.shape[0], 10)
   xpix_stone_world, ypix_stone_world = pix_to_world(xpix_stone,ypix_stone,Rover.pos[0],
                                                     Rover.pos[1],Rover.yaw,Rover.worldmap.shape[0], 10)      
   xpix_path_world, ypix_path_world = pix_to_world(xpix_path, ypix_path, Rover.pos[0],
                                                     Rover.pos[1],Rover.yaw,Rover.worldmap.shape[0], 10)
   ```

**5)Update Rover worldmap**   

   ```python
   #Update Rover worldmap (to be displayed on right side of screen)
   Rover.worldmap[ypix_rock_world, xpix_rock_world, 0] += 1
   Rover.worldmap[ypix_stone_world, xpix_stone_world, 1] += 1
   Rover.worldmap[ypix_path_world, xpix_path_world, 2] += 1
   ```
    
    
**6)Update Rover pixel distances and angles**       

   ```python
   # Update Rover pixel distances and angles
   dist, angles = to_polar_coords(xpix_path, ypix_path)
   Rover.nav_dists = dist
   Rover.nav_angles = angles
   ```
    

**7)Calculate the angle under which we will move**       

   ```python
   angle_mean = np.mean(Rover.nav_angles* 180/np.pi)
   Rover.steer = np.clip(angle_mean, -15, 15)
   ```    

**8)Move**    

   ```python
   commands = (Rover.throttle, Rover.brake, Rover.steer)
   send_control(commands, out_image_string1, out_image_string2)
   ```   




