# Herbert_Robotic_Assistant

## Purpose
Herbert is a mobile object retrieval robotic assistant designed to improve the independece of ALS patients. Herbert can pick up a range of dropped objects and return them to a user seated in a wheelchair.

## System Overview
Herbert's movement across the floor is driven by an iRobot Create 2. It takes joystick input from a PS4 controller to navigate to the dropped object. There is a laser which shines a point to ground in front of the Create; this is a guide to help the user know where Herbert is pointing. Then, Herbert utilizes our computer vision software to automatically reorient itself to pick up the object which is nearest to the laser. The object retrieval is enabled by an articulated arm mounted upon the Create, which has a dust-pan-and-sweeper-like end effector. The joystick and camera inputs are sent to a Raspberry Pi 4 running ROS. The ROS nodes which handle these inputs send commands to the Create and to a Teensy 2.0 microcontroller. The Teensy takes in angular positions and controls the motors to bring the articulated arm to the recieved angular positions.

## Software Architecture
Herbert is controlled through ROS. The ROS architecture is shown in the image below:
![ROS_Architecture](/images/ROS_Architecture.png)

### Movement
Herbert's movement is enabled by an iRobot Create 2. The create_autonomy node takes in a Twist command and moves the Create at the given speed.
...

### Computer Vision
The computer vision is powered by OpenCV and Tensorflow. We first create bounding boxes around objects in the camera frame 
...

### Arm Motor Controller
The motors and sensors in the articulated arm and end effector are driven by a Teensy 2.0 Microcontroller.

The teensyduino is subscribed to the ***topic name*** topic. This topic publishes a ***message name*** message which carries information about the angular position of each motor. The articulated arm is moved with two high torque DC motors connected to the lower two joints, and two 360 degree continuous servo motors connected to the upper two joints. All of these motors are controlled by a PID algorithm which takes the position as an inputs and outputs an analog voltage to control the motors. There is an additional 180 degree servo motor mounted on the end effector which enables the sweeping motion for object retrieval. This servo is controlled with the Arduino servo motor library. 

In addition to the motors the teensyduino also monitors Herbert's sensors. There are two pressure sensors, one on the top side of the end effetor pan, and one on the bottom. The serve the purposes of detecting when an object is on the pan and when the pan has touched the ground, respectively. There are also two IR sensors. One is mounted on the Create (level with the base of the arm), and the other is mounted on the end effector. They both serve the purpose of collision prevention. When either the base of the Create or the end of the arm is too close to a nearby obstacle, Herbert will freeze and the user must reverse to enable movement again. 
