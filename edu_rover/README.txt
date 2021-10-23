Welcome to the Edu_Rover pkg

Info:
	The Edu_Rover pkg uses ROS1
	for a list of useful commands please see (Github repo/examples)
Dependencies:
	*ROS1 noetic
	*Python3
	*Gazebo
	
1: Getting Started
	To get Started with the Edu_rover (assuming you have the dependencies and a catkin_ws setup)
	Open a new terminal and run:
	
	$roscore
	
	Open up another terminal and follow these next steps.
	Move the pkg into your catkin_ws/src folder and use catkin_build function in from the terminal to install:
	
	$cd /catkin_ws/src
	$mv /edu_rover /home/catkin_ws/src
	$cd ..
	$catkin build edu_rover
	$source devel/setup.bash
	
	To check to see if the pkg was installed correctly run:
	
	$roscd edu_rover
	
	and you should be prompted with
	Then create a catkin_ws see:
	
	$/home/catkin_ws/src/edu_rover
	
	Congradulations! edu_rover pkg has successfully been installed!
	
2: Test Run
	To launch the edu rover make sure you have a roscore already started:
	
	Open a new terminal and run:
	
	$roscore
	
	Open a new terminal to launch the edu_rover:
	
	$cd catkin_ws
	$source devel/setup.bash
	$roslaunch edu_rover edu_rover.launch
	
	This should spawn the edu_rover inside a gazebo world.
	Open a new terminal and try out the test feature:
	
	$cd catkin_ws
	$source devel/setup.bash
	$rosrun edu_rover edu_rover_test.py
	
	You should see the edu_rover moving in a square and balance off a wall.
	
3: Test edu_rover and connect to the real one.
	(To-Do)
	First we must ssh into our beaglebone and setup a ros connection across the bealgebones wifi server.
	
	$ssh 168.17...
