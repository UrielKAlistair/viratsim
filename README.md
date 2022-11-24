# viratsim
The first "kind of a big deal" abhiyaan task. 
This package simulates the vehicle virat in a virtual environment.

Running this package:

You must already have Rviz and Gazebo installed in your system in order run this package.
In addition, you must have gazebo-ros integration, and move_base to run the navstack.
I have included the install commands for the last two in a prerequisites.bash file in this directory.

Assuming you've got the dependencies, running this is fairly simple.
Simply clone my entire repo inside the src folder of a catkin workspace. The package name will be viratsim.
virat.launch will launch a gazebo world, spawn the vehicle in it and open Rviz for you to visualise anything we may be seeing.

teleop.launch will let you control the vehicle with a teleop.

move_base.launch will enable autonomous movement of the vehicle using the ROS Navigation Stack.


A detailed explanation of the function each file in this package:

(I) description

The description folder contains the urdf file for our robot, virat. 
The robot contains the following parts:

1) base_link : a dummy link with no properties. (Created because root must not have inertia for some compatibility)
2) body : The main body of virat. The Geometry and the collision is given by an imported mesh. Same goes for wheels.
3) front_wheel: A fixed castor wheel at the front
4) rear_wheel_1,2: Two rear wheels that can only rotate along the axis of the axle joining them
5) camera : A box in which a camera plugin is placed

And finally a differential drive plugin is added.
The masses and the inertias are bogus. Same goes for the surface coefficients. These parameters can all be found at the top of the urdf. 
These parameters were overly tweaked just to get the bot to not do crazy things.

(II) launch

The function of the launch files was already described in the "running this package" section.
The gazebo_world.launch opens up a gazebo world called grassy.world, which can be found in the worlds directory. It's called by virat.launch, so do not use this file individually yourself.

(III) meshes

The meshes folder merely contains the body and wheel meshes used in the urdf.

(IV) models

models contains the grassy_plane,sun and pothole models used to make the grassy.world. 
Even the boilerplate model of the sun is here because in order to contain all that we use inside the package, we have modified the gazebo model path to point inside our package in our package.xml file.

(V) config

This contains the configuration files for use by move_base, and the rviz config file.

(VI) scripts

Scripts contains 2 python files.
The teleop is fairly straightforward. It is a heavily stripped down and highly re-written version of a teleop script I found on the internet. 
It uses the termios and tty modules to convert key presses to usable python data. This is the only part of the code I didnt modify.
The key presses are mapped to linear and angular velocity parameters, and it is published to cmd_vel, a topic created by the differential drive.

The pothole detector first subscribes to /virat/camera/image_raw, a topic published to by the camera plugin. 
It first shows the camera feed in an opencv dialog box just to give a nice first person perspective when controlling the vehicle using the teleop.
It takes the camera feed, puts a gray filter on it, and then thresholds the image to make any point below a certain whiteness completely black, and the rest totally white. This works because the potholes are a perfect white.

Once this filter is applied, the potholes are the only things left in the midst of the darkness. A simple contour detection gets the boundaries we need.
This is now a set of 2D points on the image we received.

We then transform the 2D pixel coordinates into points on the ground plane.
This is then published as a point cloud, and can be visualised in RViz.

(VII) worlds

contains the gazebo world we will be operating in.
