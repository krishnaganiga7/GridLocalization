# Grid Localization Using Bayes Filter
CSE 568 Programming Assignment

Grid Localization using Bayes filter is a variant of Discrete Bayes Localization. 

The problem consists of the following steps:

1. Read the ROS bag.
2. Write functions to convert a robot pose from discrete to continuous and vice-versa
3. Define a function for odom motion model
4. Define a function for sensor model
5. Implement Bayes filter
6. Use the Rviz helper functions to display the trajectory of the robot using the beliefs calculated in your Bayes filter (the most probable state is the grid cell with the highest probability). This will also help you debug your code and tune your noise parameters.

