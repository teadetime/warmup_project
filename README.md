# A ROS Warmup _Nathan Faber
### Project Overview
This project serves as an intro point to ROS. The objective is to implement several behaviors of increasingly complex behavior. Each behavior is slightly different and will utilize a slightly different type of control system.

### Tele
I implemented a very simple teleop strategy. The neato is controlled by 5 buttons (the arrow keys and the space bar). The arrow keys cause you to increase angular velocity to increase linear velocity. This was a design choice because I wanted it to behave differently than the stock Tele-Op. The space serves as a complete stop (for both linear and angular velocity) This was different than using fixed speeds for turning and driving which allows for more complex and fun driving.

### Drive a 1Mx1M Square
In this behavior the robot should end exactly where it started-driving a sequence of repeated drive and turn combinations. My strategy was to utilize that these behaviors are repeated a known amount of times (4) to turn into a square. This required the development of a rudimentary state machine with "Start", "driving", and "turning" states. The change between these states is accomplished by using a rostime based control logic since linear and angular velocities are set in m/s. This is far from a fool-proof method (using odometry would be more stable), but the structure of the code would remain the same.

### Wall Following
Wall following is much more interesting/complex behaviour than both the tele-op and square. The goal of this behavior is to move the robot parallel to a wall without running into it. The predominant way to do this is to use the lidar scanner on the Neato to obtain data about its surroundings There are a multitude of ways that this could be implemented. My first pass utilizes a very small amount of data and is very simple. By taking 2 lidar points at 90 degrees apart from each other on the side of the robot we can determine our orientation relative to the wall. Essentially: if the front lidar distance is smaller than the back one then the robot needs to turn away from the wall and vice-versa.
One important trade off in this method is that because not much data is being used to dictate the behaviour of the robot-it is more susceptible to errors. For instance if either of the lidar values at those points is unreadable for some reason, whacky behaviour could ensue. Another shortcoming of this method is that it only looks at the differece between the two lidar measurements so it will not maintain a specific distance from a wall. This approach is also unlikely to successfully navigate corners.

### Person Follower
Person Following is very similar to wall following. Instead of maintaining distance from an object the goal is to navigate towards an object. For our case a person is defined as some object in the front half of the robot. Both the wall follower and the person follower use the lidar and will calculate a correction based on the that data. The unique thing about the person follower is that the thing you are following may not be in a known location relative to the neato. To address this I look at lidar data only in the front half of the robot.
My implentation uses a super simple algorithm that looks simply for the closest lidar point and calculates a linear speed off of it's distance, and an angular velocity based on it's angle relative to the neato. This method works very well for large "people". However, in a scenario with noise, it is likely that the neato would not follow a person well. To combat this it would likely be much more robust to look for groupings of data points or do a COM calculation to get rid of extraneous points in the lidar scan.

### Obstacle Avoidance
Obstacle avoidance is by far the most interesting behavior described here. My implementation of this behavior takes an input coordinate to navigate to without running into any objects in the way. The robot then attempts to drive there but navigates around objects that enter it's path. My method of implementation for this method is actually quite simple and handles a variety of obstacles. My approach works by first calculating three different things: ideal angle change to point towards destination point, Whether there is an obstacle to the left of the robot, an obstacle to the right, and the distance for both of those. Then the control logic is quite simple. The robot attempts to turn towards the destination as long as there is no obstacle on that side of the robot. If there are obstacles on both sides of the robot it turns away from the one that is closest to the robot. This yields a unique behavior that attempts to maintain equal distance between objects. I was surprised by how versatile this method was to a variety of setups.

The simplification into a right and left obstacle from (detected in [0-45] degrees on either side) and then averaging the distance of all those points is essential. It does require a lot of processing during each loop and likely could be optimized further. THis method also works well for straight on objects as it will choose one direction.. In addition to this the angular velocity is scaled based on how close the obstacle is to the neato this is very helpful in reducing collisions.

One downside to my approach is that if the neato is stuck behind a semi circle object or set of objects the neato will be un able to "escape". This is a relatively small problem and could be solved by using memory of paths that the robot has taken or storing LIDAR data to build a map of the world.


### Finite State Controller
As mentioned earlier, I used a simple state machine for the wall following behavior. The goal of a state machine is to transition between 2 or more states at a reproducible point (ie. arriving at destination or bumping into something). This state machine chains together more complex behaviors. At first it starts by navigating to a certain point in space while avoiding obstacles. When it reaches that point it transitions into a wall following state.following a wall and then transitions into an obstacle avoidance state in which it moves away from nearby objects while moving forward. The stop state is achieved when the robot runs into something or an arbitrary timer has run out.

### Code Structure
The code is implemented in an OOP style. The main robot loop and associated parameters are all defined in one object. This includes several ROS subscribers. Each subscriber has a processing function that stores relevant data/simple processing into the object's variables. Other helper functions are defined so that they can be used in the main _run()_ loop. The general code structure is that of a state machine, that has at minimum a _start_ and _end_ state. This is used to ensure that the robot is started and stopped correctly.

### Challenges
- Using a simulator can be tough! A lot of time was spent getting familiar with it and various problems/bad states that were gotten into.
- Odometry was stored in Quaternion and wasn't useful in it's format so it had to be converted
- Obstacle avoidance is hard, specifically it is hard to determine what constitutes and obstacle relative to the robot.

### Improvements
I've pointed out gaps/use cases that could occur in each of these behaviors. In short it seems that the way to improve all of these is to process and utilize more data/processing. This could include using a RANSAC algorithm to locate specific objects.
There is also significant processing going on in the object avoidance behavior that has not been optimized at all. There are likely improvements that could be made to that algorithm as well as simple things like using more data points from lidar and averaging to create more stable behaviors.

If I had more time I would rewrite things into a prebuilt state machine library and utilize odometry in the drive square and wall behaviors. I would also make all behaviors more flexible in starting positions as well as things like being able to wall follow on both sides.

### Takeaways
- __Have simple start and stop states:__ This can help structure the code nicely and ensure proper starting and ending conditions
- __Ensure that data is stored correctly before processing:__ Several times I did invalid storage of data or I stored data that represented something different than I intended (like the complement of an angle or a reversed sign of an angle)
- __Design code in small testable chunks:__ At points it was hard to test my more complex behaviors because there was so much going on. By breaking the problem into certain states that are testable it is easier to debug programs
- __Scale velocities according to world:__ All of my behaviors performed better when velocities were scaled proportionally to sensor data. I also noticed that certain velocities just didn't work (simply too fast to react smoothly). I found that development and behavior was more consistent at lower speeds.
- __Plan out high level behavior/processing before implementing:__ In most cases it was much more time efficient to work from the "outside in" in terms of robot control. Having an idea of the outer framework and transition between states as well as what the inner data that was driving those changes was much more efficient than simply going for it.


