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

### Wall Following

### Person Follower

### Finite State Controller

### Challenges
- Using a simulator can be tough! A lot of time was spent getting familiar with it and various problems/bad states that were gotten into.
- Odometry 

### Improvements

### Takeaways
- *Use Bag Files* Using bag files 
