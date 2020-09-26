# warmup_project_Nathan Faber


    For each behavior, describe the problem at a high-level. Include any relevant diagrams that help explain your approach.  Discuss your strategy at a high-level and include any tricky decisions that had to be made to realize a successful implementation.
    For the finite state controller, what was the overall behavior. What were the states? What did the robot do in each state? How did you combine and how did you detect when to transition between behaviors?  Consider including a state transition diagram in your writeup.
    How was your code structured? Make sure to include a sufficient detail about the object-oriented structure you used for your project.
    What if any challenges did you face along the way?
    What would you do to improve your project if you had more time?
    What are the key takeaways from this assignment for future robotic programming projects? For each takeaway, provide a sentence or two of elaboration.




# A ROS Warmup

### Tele
I implemented a very simple teleop strategy. The neato is controlled by 5 buttons (the arrow keys and the space bar). The arrow keys cause you to increase angular velocity to increase linear velocity. This was a design choice because I wanted it to behave differently than the stock Tele-Op. The space serves as a complete stop (for both linear and angular velocity) This was different than using fixed speeds for turning and driving which allows for more complex and fun driving.

### Drive a 1Mx1M Square
In this behavior the robot should end exactly where it started-driving a sequence of repeated drive and turn combinations. My startegy was to utilize that these behaviors are repeaded a known amount of times (4) to turn into a square. This required the development of a rudimentary state machine with "Start", "driving", and "turning" states. The change between these states is accomplished by using a rostime based control logic since linear and angular velocities are set in m/s. This is far from a fool-proof method (using odometry would be more stable), but the structure of the code would remain the same.

### Wall Following
Wall following is much more interesting/complex behviour than both the tele-op and square. The predominant way to do this is to use the lidar scannner on the Neato to obtain data about its surroundings There are a multitude of ways that this could be implemented. My first pass utilizes a very small amount of data and is very simple. SEE IMAGE Below. By taking 2 points at 90 degrees apart from each other out the side of the robot we can determine our orientation relative to the wall aka if the front lidar distance is smaller than the back one then the robot needs to turn away from the wall and vice-versa. 
One important trade off in this method is that because not much data is being used todictate the behaviour of the robot-it is more susceptible to errors. For instance if either of the lidar values at those points is unreadable for some reason, whacky behviour could ensue. Another shortcoming of this method is that it only looks at the differece between the two lidar measurements so it will not maintain a specific distance from a wall. This approach is also unlikely to successfully navigate corners.

### Person Follower
Person Following is very similar to wall following. Both use the lidar and will calculate a correction based on the that data. The unique thing about the person follower is that the thing you are following may not be in a known location relative to the neato. To address this I look at lidar data only in the front half of the robot.
My first implentation uses a super simple algorithm that looks simply for the closest lidar point and calculates a linear speed off of it's distance, and an angular velocity based on it's angle relative to the neato. This method _____ However, in a scenario with noise, it is likely that the neato would not follow a person well. To combat this it would liekly be much more robust to look for groupings of data points or do a COM calculation to get rid of extraneous points.

### Obstacle Avoidance
Obstacle avoidance is by far the most interesting behavior described here. My implentation of this behavior takes an input coordinate to navigate to. The robot then attempts to drive there but navigates around objects that enter it's path. My method of implentation for this method is actually quite simple and handles a variety of obstacles. My approach works by first calculating three different things: ideal angle change to point towards destination point, Whether there is an obstacle to the left of the robot, an oobstacle to the right, and the distance for both of those. Then the control logic is quite simple. The robot attempts to turn towards the destination as long as there is no obstacle in that side of the robot. If there are obstacles on both sides of the robot it turns away from the one that is closest to the robot. This yields a unique behavior that attempts to maintain equal distance between objects. I was surprised by how versatile this method was to a variety of setups. 

The simplification into a right and left obstacle from (detected in [0-45] degrees on either side) and then averaging the distance of all those points is essential. It does require a lot of processing during each loop and likely could be optimized further. THis method also works well for straight on objects as it will choose one direction.. In addition to this the angular velocity is scaled based on how close the obstacle is to the neato this is very helpful in reducing collisions.

One downside to my approach is that if the neato is stuck behind a semi circle object or set of objects the neato will be un able to "escape". This is a relatively small problem and could be solved by using memory of paths that the robot has taken or storing LIDAR data to build a map of the world.


### Finite State Controller
As mentioned earlier I used a simple state machine for the wall following behavior. However, this state machine chains together more complex behaviors. At first it starts by navigating to a certain point in space while avoiding obstacles. When it reaches that point it transitions into a wall following state.following a wall and then transitions into an obstacle avoidance state in which it moves away from nearby objects while moving forward. The stop state is achieved when the robot runs into something or an arbitrary timer has run out.

### Code Structure
The code is impleented in an OOP style. The main robot loop snd associated parameters are all defined in one object. This includes several ROS subscribers. Each subscriber has a processing function that stores relevant data/simple processing into the object's variables. Other helper functions are defined so that they can be used in hte main _run()_ loop. The general code structure is that of a state machine, that has at minimum a _start_ and _end_ state. This is used to ensure that robot is started up correctly and stopped correctly.

### Challenges
- Using a simulator can be tough! A lot of time was spent getting familiar with it and various problems/bad states that were gotten into.
- Odometry was stored in Quaternion and wasn't useful in it's format so it had to be converted
- Obstacle avoidance is hard, specifically it is hard to determine what constitutes and obstacle relative to the robot.

### Improvements
I've pointed out gaps/use cases that could occur in each of these behaviors. In short it seems that the way to improve all of these is to process and utilize more data/processing. This could include using a RANSAC algorithm to locate specific objects.
There is also significant processing going on in the object avoidance behavior that has not been optimized at all. There are likely improvements that could be made to that algorithm as well as simple things like using more data points from lidar and averaging to create more stable behaviors.

If I had more time I would rewrite things into a prebuilt state machine library and utilize odometry in the drive square and wall behaviors.

### Takeaways
- *Use Bag Files* Using bag files can really help speed up development
