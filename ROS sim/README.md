
Copy  folder's content to catkin_ws/src

run catkin_make

to run gazebo simulation :
roslaunch putarm_gazebo putarm.launch

to run rviz :
roslaunch putarm_moveit_config putarm_moveit_planning_execution.launch sim:=true limited:=true

to run c++ demo :
rosrun control_ur3sim control_ur3sim 


