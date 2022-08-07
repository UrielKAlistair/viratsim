# viratsim
The first "kind of a big deal" abhiyaan task. Involves creating a model of virat, obtaining its camera feed from a simulated environment
and manipulating the feed to deduce information.

A detailed description:

(I)
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

(II)
The launch folder contains a boilerplate getworld.launch from gazebo's files and virat.launch, which basically sets off our entire project.
The getworld opens up a gazebo world called grassy.world, which can be found in the worlds directory.

The virat.launch does the following:
1) Opens up grassy.world in Gazebo
2) Spawns the urdf in the world
3) Opens up a joint state publisher
4) And a robot state publisher. 
(I assumed that a robot state publisher would automatically publish the states of the joints, but my transforms all failed when the JSP was not initiated.Presumably, running both is the source of a spam of redundant transform warnings, but a functioning model is all I was after.)
5) Opens up RVIZ 
6) Initiates a teleop node, and opens up a new terminal for it. The new terminal is because the original one is flooded with the warnings.
7) Initiates a potholedetection node.

(III)
The meshes folder merely contains the body and wheel meshes used in the urdf.

(IV)
models contains the grassy_plane,sun and pothole models used to make the grassy.world. 
Even the boilerplate model of the sun is here because in order to contain all that we use inside the package, we have modified the gazebo model path to point inside our package in our package.xml file.

(V)
Scripts contains 2 python files.
The teleop is fairly straightforward. It is a heavily stripped down and highly re-written version of a teleop script I found on the internet. 
It uses the termios and tty modules to convert key presses to usable python data. This is the only part of the code I didnt modify.
The key presses are mapped to linear and angular velocity parameters, and it is published to cmd_vel, a topic created by the differential drive.

The pothole detector first subscribes to /virat/camera/image_raw, a topic published to by the camera plugin. 
It first shows the camera feed in an opencv dialog box just to give a nice first person perspective when controlling the vehicle using the teleop.
It takes the camera feed, puts a gray filter on it, and then thresholds the image to make any point below a certain whiteness completely black, and the rest totally white. This works because the potholes are a perfect white.

Once this filter is applied, the potholes are the only things left in the midst of the darkness. A simple contour detection gets the boundaries we need.
This is now a set of 2D points on the image we received.

We utilise some method (either the pinholecamera module or an actual matrix multiplication to avoid the datatype conversion fiasco; which one will be known before the last commit only.) to transform the 2D pixel coordinates into points on the ground plane.

This is then published as a point cloud, and it can be visualised in RViz.
