# ROS-Hydro-Ported-Caltech-Lanes-Code
This code is edited version of code written by Mohamed Aly California Institute of Technology for road detection.

The code written by Mohammed Aly uses images from a data set. I have made enough changes to subscibe images from ROS(Hydro) and process them.
Extra addons are:</n>

1. Ported it to ROS(Hydro) and converted some cv::Mat type of images we get in ROS to IplImages.
2. Calculate the distance of point infront of robot(optical center projected downwards) from the lane(equation calculated by taking two points on the lane)
3. If right side distance is more than the left then move towards right and viz.
