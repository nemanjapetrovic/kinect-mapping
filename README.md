# Team members

- [Nikola Vitanović](https://github.com/NVitanovic)
- [Mihajlo Nikolić](https://www.linkedin.com/in/mihajlonikolic94/)
- [Risto Keković](https://github.com/Risto995)
- [Nemanja Petrović](https://github.com/nemanjapetrovic)

# Details

IDE: Visual Studio

Platform: Windows

Language: C++

College: robot.elfak.ni.ac.rs

Project: RBot - Used in our project for 2D room mapping.

# Related GitHub repositories

- [Odometry server](https://github.com/nemanjapetrovic/odometry-server)
- [WiFi remote robot controller - Android](https://github.com/nemanjapetrovic/wifi-robot-remote-controller)

# Presentation document

Hello everyone, we are students of the third year at Faculty of Electronic Engineering in Niš, at the department of Computer Science, but we are currently doing a project for the laboratory of Robotics and today we will show you what we did. Basically, the idea was that we were given a robot which needed to map its surroundings and form a map of the room and all the walls and obstacles in that space, so we were given an assignment to implement that. The robot came with its own configuration, as well as camera, some infrared sensors, as well as a complete platform for controlling the movement of the robot itself. 

What we did first was creating an easy and intuitive way of controlling the robot and being able to drive and rotate it in every direction. We had to trace the packages that are being sent to the robot in order to move it in a specific direction or rotate it for a certain angle. We also wanted to get the signal from the robot’s camera, which we did by connecting to its IP address in the local network and getting a live feed in every second. Using that data we were able to design an Android application which had an easy to use interface for controlling the robot and the best of all is that it can be downloaded on any Android phone which enables the user to control it without problems.

The main problem, however, was sensors for detecting distance of the objects. Sensors which came with the robot had a very small range, so we needed something with a better range. Laser sensors such as lidar are very expensive so we went with the Kinect sensor. Apart from its use for Xbox console, where it can detect movement, it can also calculate a distance of a certain object with its ultrasonic sensor, and it’s affordable for this kind of project. We used 2 different images that we got from Kinect, one is a standard camera that records everything within its range and the second one is ultrasonic image based on the waves that are being sent at a certain speed and the amount of passed time before those waves come back. That’s how Kinect detects what is closer and what is more distant, and that’s how it forms a depth map in which distant object are light-colored, while closer objects appear darker. When those 2 images are splashed together, they can form a 3D model which can show us in real-time the entire depth of field within its range. The only problem Kinect has is that its angle is 60 degrees wide so it needs to rotate in order to get a full awareness of its surroundings.

For our project, we needed only the two-dimensional view from the Kinect in order to put that data through the Slam algorithm. SLAM stands for simultaneous localization and mapping and we used it to draw a map of the room. What it does is that it takes all the data that is being sent from the Kinect and also the odometry (position of the robot in the room), and based on all that, it forms a map of the visible space in the room. As mentioned before, the robot only has 60 degrees of width, so in order to get a full 360-degree view, it has to be rotated and moved throughout the whole room in order to get the full information about its surroundings. As you can see in the video, we tested our robot in the laboratory and it shows how it moved and recorded all the walls and obstacles in its path. Where it detected open doors, it couldn’t reach the walls in the other room so it just drew a broad space in the distance which tells us that Kinect sensor couldn’t sense any kind of boundary within its range. 

So in the end, we ended up with this image of the room. It’s not the best one, but take into account the quality of the sensor and the fact that we only worked for about a month on it. But this just shows you how many possibilities there are with this kind of mapping. Chances are that some of you are already using it with your vacuum cleaners. Those are the ones that do the vacuuming themselves, and for that, they need to know where are the walls and other furniture in the house in order not to collide with them. Another good example would be cars with automated parking, which also scans for nearby walls or cars and corrects its route in order not to hit any of these obstacles. We expect more uses of this kind of mapping in the near future, and we will continue to develop our robot to do some even more amazing things.
