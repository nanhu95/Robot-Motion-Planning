README

All Java source files can be found in the src/assignment_motion_planning folder.
The files that are in the root folder are different environments for the robot to navigate.

The program can be run through MotionPlannerDriver.java. There are many options that can be set, outlined below:

- lines 12 and 13 set the environments for the planar robot and the robot arm, respectively; there is a normal and a hard version of each environment
- line 16 sets whether the robot is a planar robot or a robot arm; setting the boolean to false will allow the user to use a robot arm
- line 22 sets the resolution to be used in the motion planner; using a larger value, such as 0.1, will allow for faster runtime, and using smaller values will increase runtime but also increase accuracy
- lines 29 and 30 set the start and goal positions, respectively, for a planar robot
- line 31 though 34 present different types of planar robots with different drive trains; leave only one line uncommented at a time
- lines 38 adn 39 set the start and goal positions, respectively, for a robot arm
- line 40 sets the number of segments in the robot arm