# Aruco-Localization

ROS package to localize a robot using an aruco tag and then move it with respect to the frame of the camera positioned overhead.

## Usage
catkin_make the package and then follow these steps:

1. This code uses an older version of opencv-contrib-python

       pip install opencv-contrib-python==4.6.0.66

 


2. The **aruco_5x5_100_1_rotated.png** image should be pasted inside **usr/share/gazebo-11/media/materials/textures** (Ubuntu 20.04) which can be accessed using nautilus.
   Then go ahead and add this code to **usr/share/gazebo-11/media/materials/scrips**:

   
       material Gazebo/aruco
         {
           receive_shadows on
      
        technique
        {
          pass
          {
            ambient 1 1 1 1.000000
            diffuse 1 1 1 1.000000
            specular 0.03 0.03 0.03 1.000000 
            emissive 0.900000 0.900000 0.900000 1.000000
      
            texture_unit
            {
              texture aruco_5x5_100_1_rotated.png
            }
          }
        }
       }

4. Scripts:

       roslaunch m2wr_description spawn.launch
   After giving executable permissions to both the files :-
       ```rosrun m2wr_description detect_aruco.py```
       ```rosrun m2wr_description runrobot.py <x> <y>```


6. Visualize the odometery published in rviz under aruco/odometery
  
