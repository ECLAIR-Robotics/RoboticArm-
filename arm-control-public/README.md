# Arm Control Mini-Project
This is an introductory project intended to familiarize yourself with controlling robotics and basic Python commands. 

## Documentation

### Controller Class
You will control the LeArm through primarily through the Controller class. 

Many member functions will ask for a "servo id" as an input. You reference a servo either through its integer id or servo name. Servo names and integer ids are listed below:

* 1: claw
* 2: hand
* 3: wrist
* 4: elbow
* 5: shoulder
* 6: base


For each servo, you are either able to set its position or angle. The position is a value where 0 and 1 are the two extremes that the servo can be set at. Each servo has a "home" posiion which is by default defined as halfway between its minimum and maximum value (i.e. 0.5).


The servo's angle is a value between -90 and 90 degrees where 0 degrees is equivalent to the home positon. Notice, depending on the home position, it is possible to specify an angle that is beyond the range of the servo. In this case, the arm will move as far as possible.

### Class Functions

`class LeArm.Controller(home=False, port='COM3', debug=False)`

Instantiates a Controller class object

* home (bool): Whether to immediately set the arm to its initial position.
* port (str): The name of the port that communicates with the Arduino (either COM3 or COM4)
* debug (bool): Whether to print additional debugging info

`home(action_time=0.5)`

Resets the arm to its "home" position. By default, this is the position at which the servo positions are halfway between their mininum and maximum values.

* action_time (float): time in milliseconds the arm will take to go the home position

`setAngle(servo_id, action_time=1000, angle=0)`

Moves one servo to a particular angle. 0 degrees is defined as the angle of that servo at the "home" position. 

* servo_id (str or int): Either the integer id or name of a servo
* action_time (float): time in milliseconds for the arm to move the servo
* angle (float): the target angle in degrees

`setPosition(servo_id, action_time=1000, position=0.5)`
Moves one servo to a particular position (Remember 0 is the min and 1 is the max value).

* servo_id (str or int): Either the integer id or name of a servo
* action_time (float): time in milliseconds for the arm to move the servo
* position (float): the target position value to move the servo 

`getPosition(servo_id)`
Gets the position of a servo

* servo_id (str or int): Either the integer id or name of a servo

`setPositions(self, action_time=1000, **movements)`
Move multiply servos at once to a set of servo positions

* action_time (float): time in milliseconds for the arm to move to the target position
* movements: Keyword arguments of the form `servo_id=position`

`readPositions(self, *servo_ids)`
Get multiple servo positions at once

* servo_ids (int or str): The integer ids or names of the servos given as arguments (e.g. readPositions(1, "base", "wrist")) 


### Camera Class

`class Camera.Webcam(webcam_id=0)`
Instantiates a Controller class object

* webcam_id (int): integer id for your webcam. It's typically 0 if you have one webcam. If not, just keep trying other integers after 0.

### Class Functions

`capture()`
Returns a H * W * 3 array representing the current frame of the webcam. The first 2 dimensions are height and width and the third dimension is the RGB (i.e. red-green-blue) color.  
