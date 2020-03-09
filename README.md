# UdacityProject5
Robotic Software Engineer Udacity Project 5: "Home Service Robot"

## The Parts
- SLAM
- navigation
- add_markers
- pick_objects

## Construction
This program initializes a ROS Node called **gmapping**, which is capable of detecting objects around the Turtlebot and generating an occupancy map based on its limited information. Next, a node called **AMCL** (Adaptive Monte Carlo Localization) is used to predict the robot's location based on the map generated and the current values of relative object locations. Thirdly, the **add_markers** node places an object in RVIZ that waits for the robot to come pick it up in the same location in Gazebo. To do this, the final node **pick_objects** sends the robot to the specified location where the marker was placed. Reaching the location causes the marker to be removed and a new marker to be placed in another location, which the robot now travels to using the same pick_objects node. It reaches the destination and "unloads" by waiting in that location.

All of these nodes working together are capable of creating a simulated "Home Service Robot" that can travel to desired locations and pretend to pick up objects.