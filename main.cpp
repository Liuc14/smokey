#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/misc.hpp"
#include <cstdlib>

pros::Controller controller(pros::E_CONTROLLER_MASTER);
// left motor group
pros::MotorGroup left_motor_group({-20,-19,-18});
// right motor group
pros::MotorGroup right_motor_group({11, 12,13});

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              9.75, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, 
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 8 
);

// imu
pros::Imu imu(16);
//vertical odom tracker
pros::Rotation vertical_tracker(-17);
//lift rotation sensor
pros::Rotation liftRotationSensor(4);
//wall distance sensor for distance resets
pros::Distance distanceFront(6);
pros::Distance distanceRight(15);
pros::Distance distanceLeft(8);
pros::Distance distanceBack(7);
//distance sensor on claw for auto clamp
pros::Distance distanceClaw(14);
//side roller motor constructor
pros::Motor rollerIntake(0, pros::MotorGears::green);
pros::Motor frontIntake(0, pros::MotorGear::blue);
//claw intake motor constructor
pros::Motor clawRoller(0, pros::MotorGears::blue);
//double lift motor constructor
pros::MotorGroup lift({-1, 2, 10}, pros::MotorGears::green);
//double flipper motor constructor
pros::Motor flipper(3, pros::MotorGears::green);
//piston to open claw for grabbing cups and pins
pros::adi::DigitalOut clawPiston('A', false);
pros::adi::DigitalOut togglePiston('B');
pros::adi::DigitalOut rollerPiston('C');
//macro definition to shorten the call for chassis.waitUntilDone for auto movements
#define waitd chassis.waitUntilDone()
//claw structure for piston commands.
struct Claw_t {
    void open() {
clawPiston.set_value(false); 
    }
    void close() {
clawPiston.set_value(true);            
         }



};

Claw_t Claw; //defining the structure and name of the object.



// horizontal tracking wheel
//lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_tracker, lemlib::Omniwheel::NEW_2, -5.75);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_tracker, lemlib::Omniwheel::NEW_2, 0);

// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel,//verticle odom tracker
                            nullptr, 
                           nullptr,
                            nullptr,
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3.5, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              400, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              18, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              400, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

//distance sensor offset based off position relative to robot center
float offsetFront = 1;
float offsetBack =4;
float offsetLeft = 1;
float offsetRight = 3.4;

//distance reset function using trigonometry to allow for distance sensor resets on 90, 180,270, and 0 angle to reset robot position
void reset(pros::Distance sensor, float sensorOffset, float headingOffset) {
	float halfField = 70.25; // half of the field width in inches

	// get distance and convert mm to inches
	float dist = sensor.get_distance() / 25.4f; 

	if (dist < 0 || dist > 200) return; // check if the distance is valid

	// Fold robot θ to [0°, 360°) before rad math (same as sensorHeading; avoids negative rad heading).
	float thetaDegForCos = static_cast<float>(chassis.getPose().theta);
	while (thetaDegForCos < 0.0f) thetaDegForCos += 360.0f;
	while (thetaDegForCos >= 360.0f) thetaDegForCos -= 360.0f;

	float heading = thetaDegForCos * 0.0174533f;
	while (heading > (1.57079632679f / 2.0f)) heading -= 1.57079632679f;

	// calculate the distance to reset
	float resetDist = (dist + sensorOffset) * cosf(heading);

	// Direction the sensor is pointing in world frame (0=+Y/top, 90=+X/right, 180=-Y/bottom, 270=-X/left).
	// Use signed angle so quadrant is correct (no fabs).
	float sh = chassis.getPose().theta + headingOffset;
	float sensorHeading = sh;
	// Fold into [0, 360). Do not use fmod here: fmod copies the sign of the dividend, so negative
	// angles stay negative; -0.0f also skips a plain "while (x < 0)" loop. While-loops are exact.
	while (sensorHeading < 0.0f) sensorHeading += 360.0f;
	while (sensorHeading >= 360.0f) sensorHeading -= 360.0f;

	// determine which wall we're facing and which axis to reset
    bool resettingX = false;
    double wallSign = 1.0;
    
    if (315 <= sensorHeading || sensorHeading <= 45) {
        // Top wall - reset Y position
        resettingX = false;
        wallSign = 1.0;
    }
    else if (45 < sensorHeading && sensorHeading <= 135) {
        // Right wall - reset X position
        resettingX = true;
        wallSign = 1.0;
    }
    else if (135 < sensorHeading && sensorHeading <= 225) {
        // Bottom wall - reset Y position
        resettingX = false;
        wallSign = -1.0;
    }
    else {
        // Left wall - reset X position
        resettingX = true;
        wallSign = -1.0;
    }
	// calculate the new position
	float newPos = wallSign * (halfField - resetDist);

	double theta = chassis.getPose().theta;
	if (resettingX) {
		chassis.setPose(newPos, chassis.getPose().y, theta);
	} else {
		chassis.setPose(chassis.getPose().x, newPos, theta);
	}
}
//individual reset functions for each distance sensor in the 4 cardinal directions(N,S,E,W)
void resetFront() {
	reset(distanceFront, offsetFront, 0);
}
void resetBack() {
	reset(distanceBack, offsetBack, 180);
}
void resetLeft() {
	reset(distanceLeft, offsetLeft, 270);
}
void resetRight() {
	reset(distanceRight, offsetRight, 90);
}

//Command wrappers for basic lemlib library movement functions 

void setPose(float x, float y, float theta) {
    chassis.setPose(x, y, theta);//sets robot position to parameters
}

void moveToPoint(float x, float y, int timeout, lemlib::MoveToPointParams params = {}, bool async = true) {
    chassis.moveToPoint(x, y, timeout, params, async); //moves robot to x,y location using a xy plane
}

void moveToPose(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params = {}, bool async = true) {
    chassis.moveToPose(x, y, theta, timeout, params, async); //moves robot to x,y,theta in boomerang style
}

void turnToAngle(float theta, int timeout, lemlib::TurnToHeadingParams params = {}, bool async = true) {
    chassis.turnToHeading(theta, timeout, params, async);//simple turn to specific angle
}

void turnToPoint(float x, float y, int timeout, lemlib::TurnToPointParams params = {}, bool async = true) {
    chassis.turnToPoint(x, y, timeout, params, async);//turns towards a point on the field
}
//lift task variables to set lift height
int liftTarget = 0;
bool manualLiftControl = false;

void liftControl() {
    //control for the actual lift movement, only activates when manualLiftControl is false, which is true except for driver period.
    if(manualLiftControl == false) {
    double kp = 0.01; //multiplier to make lift more stable by exponentially decreasing speed as it reaches target
    double error = liftTarget - liftRotationSensor.get_position();//calculate error(distance between current and target) 
    double velocity = kp * error; //sets the velocity for which the lift needs to move to reach target
    lift.move(velocity);//move until target is met.
    }
}


void liftControlTask() {
    //loops liftControl in lambda function to be repeated throughout the program(called in initialize())
    while (true) {
       liftControl();
        pros::delay(10);
    }
}

void setFlipper() {
  //  flipper.move_absolute(380, 127);
  //  pros::delay(1250);
 //   flipper.move_absolute(0, -127);
}
void flipperControl() {
 //   while (true) {
   //      if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
   // setFlipper();
 //}
 //pros::delay(10);
 //   }
}
void intake(int speed) {
    frontIntake.move(speed);
}

void clawIntake(int speed) {
    clawRoller.move(speed);
}


//Auto claw section
bool autoClaw = false;
bool slowClamp = false;
int clawDistanceRegular = 125;
int clawDistance = 125;
void autoClamp() { 
    while (true) {
        if (autoClaw) {
            const std::int32_t distance = distanceClaw.get_distance();
            if (distance >= 0 && distance <= clawDistance) {
                Claw.close();
                pros::delay(150);
                liftTarget = 20000;
                autoClaw = false;
            }
        }

        pros::delay(10);
    }
}

//set toggle state for claw intake movement for drivercontrol
bool clawState = false;
void setClawIntakeState() {
    clawState = !clawState; //sets bool value to opposite value and changes intake state based off of bool 

    if(clawState == true) {
        clawIntake(127);
    } else if (clawState == false) {
        clawIntake(0);
    }
}
//intake wrapper for clawIntake

void CloseSplitWall() {
    setPose(0, 0, 270);
  resetBack();
  resetRight();
  clawDistance = 95;
Claw.open();
chassis.moveToPoint(51, chassis.getPose().y, 750, {.maxSpeed = 40});
waitd;

autoClaw = true;
chassis.moveToPoint(62, 23.5, 450, {.forwards = false, .minSpeed = 10});
waitd;    /*   
Claw.close();
pros::delay(100);
*/

liftTarget = 40000;
clawDistance = 100;
chassis.turnToPoint(51, 4, 550, { .minSpeed = 10});
waitd;

moveToPoint(51, 4, 800);
waitd;

turnToPoint(48.5, 20, 400, {.forwards = false});
waitd;

moveToPoint(48.5, 20, 600, {.forwards = false, .maxSpeed = 70});
waitd;

liftTarget = 10000;
pros::delay(300);
Claw.open();
pros::delay(200);

liftTarget = 0;
moveToPoint(54, -14, 1000);
waitd;

turnToAngle(-23, 700, {.direction= AngularDirection::CCW_COUNTERCLOCKWISE});
waitd;

chassis.arcade(-60, -22);
pros::delay(600);
Claw.close();
pros::delay(100);
liftTarget = 50000;
turnToPoint(57, -5, 600, {.minSpeed=10});
waitd;

moveToPoint(57,-5,800);
waitd;

turnToPoint(50, -16, 500, {.forwards = false});
waitd;

moveToPoint(50, -16, 700, {.forwards = false, .maxSpeed = 100});
waitd;

liftTarget = 30000;
pros::delay(250);
Claw.open();
pros::delay(200);
moveToPoint(56, -5, 800);
waitd;
liftTarget = 5000;
turnToPoint(26,2.75, 600, {.forwards = false});
waitd;

autoClaw = true;

moveToPoint(26, 2.75, 900, {.forwards = false, .maxSpeed = 85});
waitd;
liftTarget = 62500;

moveToPoint(67, 0, 1350, {.minSpeed = 50});
waitd;
togglePiston.set_value(true);
pros::delay(200);

chassis.moveToPoint(4,5,1250, {.forwards = false});
waitd;
pros::delay(200);
liftTarget = 30000;
}
void CloseSplitLane() {
   setPose(56, 2, 90);
  resetFront();
  resetLeft();
  togglePiston.set_value(true);
  pros::delay(400);
  moveToPoint(53, chassis.getPose().y, 1000, {.forwards = false, .minSpeed = 5, .earlyExitRange = 2});
  waitd;
  togglePiston.set_value(false);
liftTarget = 30000;
  turnToPoint(49, 24, 750, {.forwards = false, .minSpeed = 10, .earlyExitRange= 10});
  waitd;

  moveToPoint(49, 24, 700, {.forwards =false, .maxSpeed = 70});
  waitd;
liftTarget = 0;
pros::delay(250);
Claw.open();
pros::delay(150);

moveToPoint(51, 12, 800);
waitd;
liftTarget =6000;
turnToPoint(25, 29.5, 500, {.forwards = false});
waitd;

moveToPoint(25, 29.5, 1100, {.forwards = false, .maxSpeed = 60});
chassis.waitUntil(19.75);
Claw.close();
pros::delay(100);
liftTarget = 12500;
waitd;

turnToPoint(48, 30, 750, {.forwards =false, .direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 5, .earlyExitRange = 10});
waitd;
liftTarget = 50000;
moveToPoint(48, 30, 900, {.forwards = false, .maxSpeed = 80});
waitd;

liftTarget = 30000;
pros::delay(400);
Claw.open();
pros::delay(200);
liftTarget = 4000;


moveToPoint(36, 29, 900, {.maxSpeed = 90});
waitd;

turnToPoint(52, 51, 600, {.forwards = false});
waitd;

moveToPoint(52, 51, 1250, {.forwards =false, .maxSpeed = 55});
chassis.waitUntil(19);
Claw.close();
pros::delay(100);
liftTarget = 20000;
waitd;
moveToPoint(28, 27, 1000);
waitd;

turnToPoint(54, -20, 800, {.forwards =false, .direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 10, .earlyExitRange = 15});
liftTarget = 75000;
waitd;

moveToPoint(54, -20, 1500, {.forwards = false, .maxSpeed = 70});
waitd;

liftTarget = 35000;
pros::delay(400);
Claw.open();
pros::delay(200);
liftTarget = 0;
 
}

void CloseElim4() {

      setPose(0, 0, 270);
  resetBack();
  resetRight();
  clawDistance = 100;
Claw.open();
chassis.moveToPoint(51, chassis.getPose().y, 750, {.maxSpeed = 30});
waitd;

autoClaw = true;
chassis.moveToPoint(62, 23.5, 450, {.forwards = false, .minSpeed = 10});
waitd;    /*   
Claw.close();
pros::delay(100);
*/

liftTarget = 45000;
chassis.turnToPoint(56, 38, 550, { .minSpeed = 10});
waitd;

moveToPoint(57, 38, 800);
waitd;

turnToPoint(48, 24, 400, {.forwards = false});
waitd;

moveToPoint(48, 24, 600, {.forwards = false, .maxSpeed = 70});
waitd;

liftTarget = 20000;
pros::delay(300);
Claw.open();
pros::delay(250);

moveToPoint(57, 38, 700);
pros::delay(200);
liftTarget = 0;
waitd;



turnToPoint(46.6, 46, 500, {.forwards = false});
autoClaw = true;
moveToPoint(46.6, 46, 600, {.forwards = false, .maxSpeed = 90, .minSpeed = 35, .earlyExitRange = 2.25});
waitd;
liftTarget =60000;

turnToPoint(48.75, 24, 600, {.forwards = false});
waitd;
autoClaw = false;
moveToPoint(48.75,24, 850, {.forwards = false, .maxSpeed = 70});
waitd;
liftTarget = 35000;
pros::delay(350);
Claw.open();
pros::delay(200);
waitd;
moveToPoint(48.75, 39, 800);
waitd;
liftTarget = 0;

turnToPoint(27,21,550, {.forwards = false});
waitd;
autoClaw = true;


moveToPoint(27, 21, 1000, {.forwards = false, .maxSpeed = 70});
waitd;


turnToPoint(48, 0, 700);
waitd;


moveToPoint(48,0, 1200, { .minSpeed = 2, .earlyExitRange = 1});
waitd;



moveToPoint(72, -2.5, 800);
liftTarget = 90000;
waitd;

togglePiston.set_value(true);
pros::delay(200);

moveToPoint(54, 0, 600, {.forwards = false, .minSpeed = 10, .earlyExitRange = 0.5});
waitd;

turnToPoint(54,24,400,{.forwards = false, .minSpeed = 10, .earlyExitRange = 4});
waitd;
moveToPoint(54, 24, 1000, {.forwards = false, .maxSpeed = 60});
waitd;
liftTarget = 45000;
pros::delay(500);
Claw.open();
pros::delay(100);

    /*
setPose(56, 2, 90);
  resetFront();
  resetLeft();
  togglePiston.set_value(true);
  pros::delay(300);
  moveToPoint(52.75, chassis.getPose().y, 1000, {.forwards = false, .minSpeed = 5, .earlyExitRange = 2});
  waitd;
  togglePiston.set_value(false);
liftTarget = 30000;
  turnToPoint(49, 24, 750, {.forwards = false, .minSpeed = 10, .earlyExitRange= 10});
  waitd;

  moveToPoint(49, 24, 700, {.forwards =false, .maxSpeed = 70});
  waitd;
liftTarget = 0;
pros::delay(250);
Claw.open();
pros::delay(150);

moveToPoint(48, 12, 800);
waitd;

liftTarget = 0;

turnToPoint(62, 24.25, 450, {.forwards = false});
waitd;

moveToPoint(62, 24.25, 1000, {.forwards =false, .maxSpeed = 60});
waitd;

chassis.arcade(-100, 20);
pros::delay(400);

Claw.close();
chassis.arcade(0,0);
liftTarget = 45000;

chassis.turnToPoint(56, 4, 600, {.minSpeed =40});
waitd;

moveToPoint(56,4, 650);
waitd;

turnToPoint(56, 24, 400, {.forwards =false});
waitd;
moveToPoint(56, 24, 700, {.forwards = false, .maxSpeed = 70});
waitd;
liftTarget = 30000;
pros::delay(200);
Claw.open();
pros::delay(200);

moveToPoint(52, 11, 800);
waitd;

liftTarget = 0;
turnToPoint(25.5, 28.75, 475, {.forwards = false});
waitd;

moveToPoint(25.5, 28.75, 900, {.forwards = false, .maxSpeed = 70});
chassis.waitUntil(15);
Claw.close();
pros::delay(100);
liftTarget = 12500;
waitd;

turnToPoint(52, 27, 750, {.forwards =false, .direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 10, .earlyExitRange = 15});
waitd;
liftTarget = 70000;
moveToPoint(52, 27, 900, {.forwards = false, .maxSpeed = 80});
waitd;

liftTarget = 50000;
pros::delay(400);
Claw.open();
pros::delay(200);
liftTarget = 0;


moveToPoint(42, 28, 700);
waitd;

turnToPoint(59, 48, 500, {.forwards = false});
waitd;

moveToPoint(59, 48, 1250, {.forwards =false, .maxSpeed = 70});
chassis.waitUntil(21);
Claw.close();
pros::delay(100);
liftTarget = 12500;
waitd;

turnToPoint(60, 30, 800, {.forwards =false, .direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 25, .earlyExitRange = 20});
liftTarget = 80000;
waitd;

moveToPoint(60, 30, 750, {.forwards = false, .maxSpeed = 70});
waitd;

liftTarget = 35000;
pros::delay(400);
Claw.open();
pros::delay(200);
liftTarget = 0;
*/



/*
moveToPoint(54, 44, 800, {.maxSpeed = 110});
waitd;

turnToPoint(66, 35.75, 500, {.forwards = false});
waitd;

moveToPoint(66, 35.75, 650, {.forwards = false});
waitd;

chassis.arcade(-100, -20);
pros::delay(400);

Claw.close();
chassis.arcade(0,0);
liftTarget = 87500;

chassis.swingToHeading(20, lemlib::DriveSide::RIGHT, 600, {.minSpeed = 80});
waitd;

moveToPoint(67, 44, 500, {.minSpeed = 40, .earlyExitRange = 2});
waitd;
moveToPoint(48, 20, 600, {.forwards =false});
pros::delay(400);
liftTarget = 60000;
waitd;


*/


}




void CloseElim() { 
 setPose(56, 2, 90);
  resetFront();
  resetLeft();
  togglePiston.set_value(true);
  pros::delay(400);
  moveToPoint(52.25, chassis.getPose().y, 1000, {.forwards = false, .minSpeed = 5, .earlyExitRange = 2});
  waitd;
  togglePiston.set_value(false);
liftTarget = 30000;
  turnToPoint(49, 24, 750, {.forwards = false, .minSpeed = 10, .earlyExitRange= 10});
  waitd;

  moveToPoint(49, 24, 700, {.forwards =false, .maxSpeed = 70});
  waitd;
liftTarget = 0;
pros::delay(250);
Claw.open();
pros::delay(150);

moveToPoint(51, 12, 800);
waitd;
liftTarget =8000;
turnToPoint(23, 29.5, 700, {.forwards = false});
waitd;
autoClaw=true;

moveToPoint(23, 29.5, 1100, {.forwards = false, .maxSpeed = 56});
/*
chassis.waitUntil(21.5);
Claw.close();
pros::delay(100);
liftTarget = 12500;
*/
waitd;

turnToPoint(48, 30, 750, {.forwards =false, .direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 5, .earlyExitRange = 10});
waitd;
liftTarget = 50000;
moveToPoint(48, 30, 900, {.forwards = false, .maxSpeed = 80});
waitd;

liftTarget = 30000;
pros::delay(400);
Claw.open();
pros::delay(200);
liftTarget = 6000;


moveToPoint(36, 29, 900, {.maxSpeed = 90});
waitd;

turnToPoint(52, 51, 600, {.forwards = false});
waitd;
autoClaw=true;
moveToPoint(52, 51, 1250, {.forwards =false, .maxSpeed = 59});
/*
chassis.waitUntil(21);
Claw.close();
pros::delay(100);
liftTarget = 20000;
*/
waitd;

turnToPoint(54, 30, 800, {.forwards =false, .direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 10, .earlyExitRange = 15});
liftTarget = 80000;
waitd;

moveToPoint(54, 30, 900, {.forwards = false, .maxSpeed = 50});
waitd;

liftTarget = 50000;
pros::delay(700);
Claw.open();
pros::delay(400);
//liftTarget = 0;


//moveToPoint(48, 48, 1000, {.maxSpeed = 80});
//waitd;   


}
void FarSplitLane() {
    setPose(2, 56, 0);
  
  resetFront();
  resetLeft();
  togglePiston.set_value(true);
  pros::delay(300);
  moveToPoint(chassis.getPose().x, 52.25, 1000, {.forwards = false, .minSpeed = 5, .earlyExitRange = 2});
  waitd;
  togglePiston.set_value(false);
liftTarget = 30000;
  turnToPoint(24, 49.5, 750, {.forwards = false, .minSpeed = 10, .earlyExitRange= 10});
  waitd;

  moveToPoint(24, 49.5, 700, {.forwards =false, .maxSpeed = 70});
  waitd;
liftTarget = 0;
pros::delay(250);
Claw.open();
pros::delay(150);

moveToPoint(10, 51, 800);
waitd;
liftTarget =0;
turnToPoint(27, 23, 500, {.forwards = false});
waitd;

moveToPoint(27, 23, 900, {.forwards = false, .maxSpeed = 60});
chassis.waitUntil(17);
Claw.close();
pros::delay(100);
liftTarget = 12500;
waitd;

turnToPoint(26, 48, 750, {.forwards =false, .direction = AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 5, .earlyExitRange = 10});
waitd;
liftTarget = 50000;
moveToPoint(26, 48, 900, {.forwards = false, .maxSpeed = 80});
waitd;

liftTarget = 25000;
pros::delay(400);
Claw.open();
pros::delay(200);
liftTarget = 0;


moveToPoint(27, 36, 850, {.maxSpeed = 90});
waitd;

turnToPoint(48.25, 53, 550, {.forwards = false});
waitd;

moveToPoint(48.25, 53, 1250, {.forwards =false, .maxSpeed = 65});
chassis.waitUntil(17);
Claw.close();
pros::delay(100);
liftTarget = 12500;
waitd;

moveToPoint(26, 26.5,1000);
waitd;

turnToPoint(-30, 59.5, 800, {.forwards =false, .direction = AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 10, .earlyExitRange = 15});
liftTarget = 80000;
waitd;

moveToPoint(-30, 59.5, 2400, {.forwards = false, .maxSpeed = 50});
waitd;

liftTarget = 40000;
pros::delay(500);
Claw.open();
pros::delay(200);
liftTarget = 0;
   
}

void FarSplitWall() {
    setPose(2,56,0);
    resetFront();
  resetLeft();
  togglePiston.set_value(true);
  pros::delay(300);
  moveToPoint(chassis.getPose().x, 52.5, 1000, {.forwards = false, .minSpeed = 5, .earlyExitRange = 2});
  waitd;
  togglePiston.set_value(false);
liftTarget = 30000;
  turnToPoint(24, 49.5, 750, {.forwards = false, .minSpeed = 10, .earlyExitRange= 10});
  waitd;

  moveToPoint(24, 49.5, 700, {.forwards =false, .maxSpeed = 70});
  waitd;
liftTarget = 0;
pros::delay(250);
Claw.open();
pros::delay(150);

moveToPoint(10, 51, 800);
waitd;
liftTarget =0;

turnToPoint(23, 65, 450, {.forwards = false});
waitd;

moveToPoint(23, 65, 1000, {.forwards =false, .maxSpeed = 50});
waitd;

chassis.arcade(-100, -20);
pros::delay(400);

Claw.close();
chassis.arcade(0,0);
liftTarget = 45000;

chassis.turnToPoint(4, 54, 600, {.minSpeed =40});
waitd;

moveToPoint(4,54, 650);
waitd;

turnToPoint(24, 50, 400, {.forwards =false});
waitd;
moveToPoint(24, 50, 700, {.forwards = false, .maxSpeed = 70});
waitd;
liftTarget = 30000;
pros::delay(200);
Claw.open();
pros::delay(200);

turnToAngle(270, 700);
liftTarget =80000;
waitd;
pros::delay(200);
resetRight();
resetBack();
liftTarget = 0;

moveToPoint(-11, 50, 1500, {.maxSpeed = 100});   
waitd;
liftTarget=0;

turnToPoint(-21, 59, 600, {.forwards = false, .direction = AngularDirection::CCW_COUNTERCLOCKWISE});
waitd;

moveToPoint(-21, 59, 1250, {.forwards = false, .maxSpeed = 60});
waitd;

chassis.arcade(-100, 20);
pros::delay(600);

Claw.close();
chassis.arcade(0,0);
liftTarget = 50000;

chassis.turnToPoint(0, 54, 700, {.minSpeed =20});
waitd;

moveToPoint(0,54, 1000);
waitd;

turnToPoint(-24, 51, 400, {.forwards =false});
waitd;
moveToPoint(-24, 51, 600, {.forwards = false});
waitd;
liftTarget = 0;
pros::delay(200);
Claw.open();
pros::delay(200);


};
void FarElim() {
     setPose(2, 56, 0);
  
  resetFront();
  resetLeft();
  togglePiston.set_value(true);
  pros::delay(300);
  moveToPoint(chassis.getPose().x, 53, 1000, {.forwards = false, .minSpeed = 5, .earlyExitRange = 2});
  waitd;
  togglePiston.set_value(false);
liftTarget = 30000;
  turnToPoint(24, 49, 750, {.forwards = false, .minSpeed = 10, .earlyExitRange= 10});
  waitd;

  moveToPoint(24, 49, 700, {.forwards =false, .maxSpeed = 70});
  waitd;
liftTarget = 0;
pros::delay(250);
Claw.open();
pros::delay(150);

moveToPoint(11, 51, 800);
waitd;
liftTarget =0;
turnToPoint(26, 25.5, 500, {.forwards = false});
waitd;

moveToPoint(26, 25.5, 900, {.forwards = false, .maxSpeed = 60});
chassis.waitUntil(15.75);
Claw.close();
pros::delay(100);
liftTarget = 12500;
waitd;

turnToPoint(26, 48, 750, {.forwards =false, .direction = AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 5, .earlyExitRange = 10});
waitd;
liftTarget = 50000;
moveToPoint(26, 48, 900, {.forwards = false, .maxSpeed = 80});
waitd;

liftTarget = 25000;
pros::delay(400);
Claw.open();
pros::delay(200);
liftTarget = 0;


moveToPoint(27, 36, 850, {.maxSpeed = 90});
waitd;

turnToPoint(48, 51, 550, {.forwards = false});
waitd;

moveToPoint(48, 51, 1250, {.forwards =false, .maxSpeed = 65});
chassis.waitUntil(17);
Claw.close();
pros::delay(100);
liftTarget = 12500;
waitd;

turnToPoint(24, 54, 800, {.forwards =false, .direction = AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 10, .earlyExitRange = 15});
liftTarget = 80000;
waitd;

moveToPoint(24, 54, 900, {.forwards = false, .maxSpeed = 50});
waitd;

liftTarget = 55000;
pros::delay(600);
Claw.open();
pros::delay(200);








}
void utilTest() {
  
    

}

void closeAwp() {
    setPose(0, 0, 270);
  resetBack();
  resetRight();
  clawDistance = 100;
Claw.open();
chassis.moveToPoint(51, chassis.getPose().y, 750, {.maxSpeed = 30});
waitd;

autoClaw = true;
chassis.moveToPoint(62, 23.5, 450, {.forwards = false, .minSpeed = 10});
waitd;    /*   
Claw.close();
pros::delay(100);
*/

liftTarget = 45000;
chassis.turnToPoint(56, 38, 550, { .minSpeed = 10});
waitd;

moveToPoint(57, 38, 800);
waitd;

turnToPoint(48, 24, 400, {.forwards = false});
waitd;

moveToPoint(48, 24, 600, {.forwards = false, .maxSpeed = 70});
waitd;

liftTarget = 20000;
pros::delay(300);
Claw.open();
pros::delay(250);

moveToPoint(57, 38, 700);
pros::delay(200);
liftTarget = 0;
waitd;



turnToPoint(46.75, 46, 500, {.forwards = false});
autoClaw = true;
moveToPoint(46.75, 46, 600, {.forwards = false, .maxSpeed = 90, .minSpeed = 35, .earlyExitRange = 2.25});
waitd;
liftTarget =45000;

turnToPoint(24, 47, 300, {.forwards = false});
waitd;
moveToPoint(24, 47, 850, {.forwards = false, .maxSpeed = 70});
waitd;
liftTarget = 15000;
pros::delay(300);
Claw.open();
pros::delay(100);
waitd;
moveToPoint(39, 47, 800);
waitd;
liftTarget = 0;

turnToPoint(23.25,26,450, {.forwards = false});
waitd;
autoClaw = true;


moveToPoint(23.25, 26, 1000, {.forwards = false, .maxSpeed = 70});
waitd;

turnToPoint(45, -32, 450, {.forwards = false});
waitd;
liftTarget = 65000;

moveToPoint(45, -32, 1400, {.forwards = false, .maxSpeed = 110});
waitd;

liftTarget = 30000;
pros::delay(400);
Claw.open();
pros::delay(200);

moveToPoint(48, -2, 900, {.maxSpeed =110});
waitd;

turnToPoint(72, -2, 500);
waitd;

moveToPoint(72, -2, 700);
waitd;

togglePiston.set_value(true);
pros::delay(200);
chassis.arcade(-70, 0);
pros::delay(400);
chassis.arcade(0,0);

}
 
void Skills() {
   // utilTest();
    autoClaw = false;
    clawDistance = 100;
//CloseElim();
//setPose(0, 0,0);
turnToAngle(0, 700);
waitd;

resetRight();
resetFront();
Claw.open();
liftTarget = 0;
pros::delay(200);
liftRotationSensor.set_position(9500);
moveToPoint(48, 60, 1000);
waitd;

turnToPoint(64, 60, 600, {.forwards = false});
waitd;

moveToPoint(64, 60, 600, {.forwards = false});
waitd;
pros::delay(350);
Claw.close();
pros::delay(100);
liftTarget = 12500;
moveToPoint(48, 60, 800);
waitd;
liftTarget = 80000;
turnToPoint(48, 29, 600, {.forwards = false});
waitd;

moveToPoint(48, 29, 1400, {.forwards = false, .maxSpeed = 60});
waitd;


turnToAngle(0, 700);
Claw.open();
pros::delay(200);
waitd;

resetRight();
resetFront();
///////////////////////////////////////////////////////////////
moveToPoint(48, 60, 1000);
waitd;

turnToPoint(64, 60, 600, {.forwards = false});
waitd;

moveToPoint(64, 60, 600, {.forwards = false});
waitd;
pros::delay(350);
Claw.close();
pros::delay(100);
liftTarget = 12500;
moveToPoint(48, 60, 800);
waitd;
liftTarget = 100000;
turnToPoint(48, 29, 600, {.forwards = false});
waitd;

moveToPoint(48, 29, 1400, {.forwards = false, .maxSpeed = 60});
waitd;


turnToAngle(0, 700);
Claw.open();
pros::delay(200);
waitd;

resetRight();
resetFront();
/*
moveToPoint(51, 60, 1250, {.maxSpeed = 110});
waitd;

turnToPoint(64, 60, 750, {.forwards = false});
waitd;

moveToPoint(64, 60, 600, {.forwards = false, .maxSpeed = 90});
waitd;

pros::delay(200);
Claw.close();
pros::delay(100);
liftTarget = 15000;
pros::delay(100);
moveToPoint(50, 60, 1250, {.maxSpeed = 100});
waitd;

turnToPoint(50, 28, 700, {.forwards = false});
liftTarget = 90000;
waitd;

moveToPoint(50, 28, 1500, {.forwards = false, .maxSpeed = 50});
waitd;

liftTarget = 70000;
pros::delay(400);
Claw.open();
pros::delay(200);
////////////////////////////////////////
turnToAngle(0, 600);
waitd;
resetRight();
resetFront();
Claw.open();
moveToPoint(51, 61.5, 1250, {.maxSpeed = 100});
pros::delay(300);
liftTarget = 0;
waitd;

turnToPoint(64, 60, 750, {.forwards = false});
waitd;

moveToPoint(64, 60, 600, {.forwards = false, .maxSpeed = 90});
waitd;

pros::delay(300);
Claw.close();
pros::delay(100);
liftTarget = 12500;
pros::delay(100);

moveToPoint(50, 60, 1250, {.maxSpeed = 100});
waitd;

turnToPoint(50, 28, 700, {.forwards = false});
liftTarget = 115000;
waitd;

moveToPoint(50, 28, 1500, {.forwards = false, .maxSpeed = 50});
waitd;
liftTarget = 85000;
pros::delay(400);
Claw.open();
pros::delay(200);
////////////
turnToAngle(0, 700);
waitd;
resetRight();
resetFront();
Claw.open();
moveToPoint(50, 59, 1250, {.maxSpeed = 110});
pros::delay(300);
liftTarget = 0;
waitd;

turnToPoint(64, 59, 750, {.forwards = false});
waitd;

moveToPoint(64, 59, 600, {.forwards = false, .maxSpeed = 90});
waitd;

pros::delay(400);
Claw.close();
pros::delay(150);
liftTarget = 7500;

moveToPoint(45, 48, 1450, {.maxSpeed = 100});
waitd;

turnToAngle(90, 700);
liftTarget = 45000;
waitd;

resetLeft();
resetFront();

moveToPoint(24, 54, 800, {.forwards = false, .maxSpeed = 70});
waitd;
liftTarget = 0;
pros::delay(400);
Claw.open();
pros::delay(200);
////////////////////////////////// NEW STACK
moveToPoint(45, 62, 1500, {.maxSpeed = 110});
waitd;

turnToAngle(270, 750);
waitd;

resetRight();

moveToPoint(61, 62, 800, {.forwards = false, .maxSpeed = 90});
waitd;

pros::delay(400);
Claw.close();
pros::delay(150);
liftTarget = 7500;

moveToPoint(48, 56, 1000, {.maxSpeed = 100});
waitd;
liftTarget = 47500;

turnToPoint(19, 56, 800, {.forwards = false, .minSpeed = 5, .earlyExitRange = 10});
waitd;

moveToPoint(19, 56, 850, {.forwards = false, .maxSpeed = 70});
waitd;
liftTarget = 25000;
pros::delay(400);
Claw.open();
pros::delay(200);
////////////////////////////
moveToPoint(45, 62, 1500, {.maxSpeed = 110});
pros::delay(300);
liftTarget = 0;
waitd;

turnToAngle(-90, 850);
waitd;

resetRight();


moveToPoint(60, 62, 900, {.forwards = false, .maxSpeed = 90});
waitd;

pros::delay(400);
Claw.close();
pros::delay(150);
liftTarget = 7500;


moveToPoint(46, 54, 1000, {.maxSpeed = 100});
waitd;
liftTarget = 75000;

turnToPoint(19, 56, 750, {.forwards = false});
waitd;

moveToPoint(19, 56, 800, {.forwards = false, .maxSpeed = 70});
waitd;
liftTarget = 50000;
pros::delay(600);
Claw.open();
pros::delay(200);
////////////////////////////////////////////////////////
moveToPoint(45.5, 62, 1500, {.maxSpeed = 110});
pros::delay(400);
liftTarget = 0;
waitd;

turnToAngle(270, 750);
waitd;

resetRight();

moveToPoint(60, 62, 800, {.forwards = false, .maxSpeed = 90});
waitd;

pros::delay(400);
Claw.close();
pros::delay(150);
liftTarget = 7500;

moveToPoint(46, 52, 1500, {.maxSpeed = 100});
pros::delay(300);
liftTarget = 105000;
waitd;

turnToAngle(90, 900);
waitd;

resetLeft();
resetFront();

moveToPoint(19, 54, 850, {.forwards = false, .maxSpeed = 50});
waitd;
liftTarget = 45000;
pros::delay(800);
Claw.open();
pros::delay(200);
///////////


moveToPoint(45, 62, 1500, {.maxSpeed = 110});
pros::delay(400);
liftTarget = 0;
waitd;

turnToAngle(270, 750);
waitd;

resetRight();

moveToPoint(60, 63, 800, {.forwards = false, .maxSpeed = 90});
waitd;

pros::delay(400);
Claw.close();
pros::delay(150);
liftTarget = 7500;


moveToPoint(50, 62, 1500, {.maxSpeed = 100});
waitd;
/*
resetRight();

turnToPoint(-11, 10, 700, {.forwards = false});
waitd;
liftTarget  = 50000;


moveToPoint(-11, 10, 1800, {.forwards = false,.maxSpeed = 110});
waitd;

liftTarget = 20000;
pros::delay(200);
Claw.open();
pros::delay(200);

liftTarget =40000;

moveToPoint(-2, 84, 1500);
pros::delay(200);
liftTarget = 0;
waitd;


togglePiston.set_value(true);
pros::delay(200);

moveToPoint(-5,0,1000, {.forwards = false, .maxSpeed = 110});
waitd;


*/




}

void closeWall3() {
 setPose(56, 2, 90);
  resetFront();
  resetLeft();
  togglePiston.set_value(true);
  pros::delay(200);
  moveToPoint(52, 28, 700, {.forwards = false, .maxSpeed =110});
  liftTarget = 25000;
  waitd;
  togglePiston.set_value(false);

liftTarget = 2000;
pros::delay(250);
Claw.open();
pros::delay(150);

moveToPoint(54, 14, 800);
waitd;

turnToPoint(58, 18.25, 500, {.forwards = false});
waitd;


chassis.arcade(-65, 25);
pros::delay(850);

Claw.close();
chassis.arcade(0,0);
liftTarget = 37500;

turnToPoint(56, 8, 550, {.minSpeed =10});
waitd;

moveToPoint(56, 8, 700);
waitd;

turnToPoint(52, 22, 500, {.forwards =false});
waitd;

moveToPoint(52, 22, 600, {.forwards = false, .maxSpeed = 80});
waitd;
liftTarget = 20000;
turnToPoint(56, -13, 750);
pros::delay(200);
Claw.open();
pros::delay(150);
waitd;

moveToPoint(56.5, -13, 1000);
waitd;

liftTarget = 2000;
turnToPoint(64, -25, 800, {.forwards =false, .direction = AngularDirection::CCW_COUNTERCLOCKWISE});
waitd;

chassis.arcade(-60, -20);
pros::delay(900);

Claw.close();
chassis.arcade(0,0);
liftTarget = 20000;


turnToPoint(58, 0, 750, {.minSpeed =20});
waitd;

moveToPoint(58, 0, 1000, {.maxSpeed =110});
waitd;

turnToPoint(57, 20, 850, {.forwards =false});
liftTarget = 60000;
waitd;

moveToPoint(57, 20, 1000, {.forwards = false, .maxSpeed = 70});
waitd;
liftTarget = 45000;
pros::delay(250);
Claw.open();
pros::delay(150);

moveToPoint(48, 4, 1000);
waitd;
turnToAngle(135, 600);
waitd;


}

void FarSplit3() {
    setPose(56, 2, 0);
  resetFront(); 
  resetLeft();
  togglePiston.set_value(true);
  pros::delay(200);
  moveToPoint(28.5, 51, 750, {.forwards = false, .maxSpeed =110});
  liftTarget = 27500;
  waitd;
  togglePiston.set_value(false);

liftTarget = 2000;
pros::delay(400);
Claw.open();
pros::delay(150);

moveToPoint(10.5, 57, 800);
waitd;

turnToPoint(19.25,61.25, 500, {.forwards = false});
waitd;


chassis.arcade(-65, -30);
pros::delay(850);

Claw.close();
chassis.arcade(0,0);
liftTarget = 40000;
turnToPoint(2, 56, 550, {.minSpeed =10});
waitd;

moveToPoint(2, 56, 700);
waitd;

turnToPoint(22, 51, 500, {.forwards =false});
waitd;

moveToPoint(22, 51, 600, {.forwards = false, .maxSpeed = 80});
waitd;
liftTarget = 30000;
turnToPoint(-17.5, 55, 750);
liftTarget = 30000;
pros::delay(300);
Claw.open();
pros::delay(150);
waitd;

moveToPoint(-17.5, 55, 1000);
waitd;

liftTarget = 2000;
turnToPoint(-30 , 60, 800, {.forwards =false, .direction = AngularDirection::CW_CLOCKWISE});
waitd;

chassis.arcade(-60, 30);
pros::delay(900);

Claw.close();
chassis.arcade(0,0);
liftTarget = 20000;


turnToPoint(0, 58, 750, {.minSpeed =20});
waitd;

moveToPoint(0, 58, 1000, {.maxSpeed =110});
waitd;

turnToPoint(20, 56, 850, {.forwards =false});
liftTarget = 67500;
waitd;

moveToPoint(20,56, 1200, {.forwards = false, .maxSpeed = 70});
waitd;
liftTarget = 40000;
pros::delay(400);

}
void pidTest() {}

//string for the names of our autonomous list, used to display which auton is selected for both controller and brain selector.
const char* autonomousNames[] = {
    "CloseSplitLANE   ",
    "CloseElim        ",
    "FarSplitWALL     ",
    "FarElim          ",
    "Skills           ",
    "closeWall3       ",
    "CloseSplitWALL   ",
    "None             ",
    "FarSplitLane     ",
    "FarSplit3",
};

//int determines which auton is ran.
int selectedAuton =6;

//case function which runs different autonomous routines based off the value of selectedAuton
void chooseAuton() {
switch(selectedAuton) {
    case 0:
        CloseSplitLane();
        break;
    case 1:
       // CloseElim();
       CloseElim4();
        break;
    case 2:
       FarSplitWall();
        break;
    case 3:
        FarElim();
        break;
    case 4:
        Skills();
        break;
    case 5:
       closeWall3();
        break;
    case 6:
    CloseSplitWall();
        break;
    case 7:
    closeAwp();
    //utilTest();
       // selectedAuton = 7;
        break;
        case 8:
        FarSplitLane();
        break;
        case 9:
        FarSplit3();
    default:
        selectedAuton = 0; // default to auton 0 if invalid selection
}
}
//touch detector on brain to figure out where we touch, determining what buttons will activate.
void screenAutonButtonPressed() {
    pros::screen_touch_status_s_t status = pros::screen::touch_status();

    if (status.y < 150) {
        return;
    }

    if (status.x < 200) {
        selectedAuton--;
        if (selectedAuton < 0) {
            selectedAuton = 7;
        }
    } else {
        selectedAuton++;
        if (selectedAuton > 7) {
            selectedAuton = 0;
        }
    }
}
//calls back regularly after every touch
void screenSelector() {
    pros::screen::touch_callback(screenAutonButtonPressed, pros::E_TOUCH_PRESSED);
}
// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    //Claw.open();
    //set chassis motor configuration
    left_motor_group.set_gearing({pros::MotorGears::blue, pros::MotorGears::blue});
    right_motor_group.set_gearing({pros::MotorGears::blue, pros::MotorGears::blue});
    liftRotationSensor.reset_position();//resets lift rotation sensor for auto use.
   // lift.tare_position();
    screenSelector(); //calling the conditions for the button touch
     pros::screen_touch_status_s_t status = pros::screen::touch_status();//defines status of brain touching position to where last touch was made.
     //information printing lambda function that prints useful information like robot position(x,y,theta), selected autonomous, and our lift position.
     //also draws out our auton selector buttons on the brain and controller.
     pros::Task screenTask([&]() {
            while (true) {
                pros::screen::erase();
                // print robot location to the brain screen
                pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM_CENTER, 0, "X: %f, Y: %f",
                                    chassis.getPose().x, chassis.getPose().y); // x
 pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM_CENTER, 1, "Theta: %f",
                                    chassis.getPose().theta); //        
                                     pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM_CENTER, 2, "Selected Autonomous: %s", autonomousNames[selectedAuton]); // selected autonomous
              pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM_CENTER, 3, "Lift pos: %d",
                                    liftRotationSensor.get_position()); // x

                pros::screen::set_pen(pros::Color::red);
                pros::screen::fill_rect(0, 170, 239, 239);
                pros::screen::set_pen(pros::Color::green);
                pros::screen::fill_rect(240, 170, 479, 239);
                pros::screen::set_pen(pros::Color::white);
                pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 82, 202, "- PREVIOUS");
                pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 322, 202, "NEXT +");
                                     controller.print(0, 14, "%s", autonomousNames[selectedAuton]);
                pros::delay(50);
            }
        });
       pros::Task liftTask(liftControlTask);//runs liftControlTask in a task to run the function is parallel to rest of program.
      
      pros::Task autoClampTask(autoClamp);
    }


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    //calls for selected autonomous routine to run
    chooseAuton();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
//variables to determine button states
bool r1, r2, l1, l2, buttonUp, buttonDown, buttonLeft, buttonRight, buttonB, buttonA, buttonX, buttonY;
bool clawIndexBool;
bool clawToggleBool;
int buttonFlagB = 1;
int intakeOldSpeed;

//drivercontrol for this program
 void driverControl() {
    manualLiftControl = true;
    r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    l1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    l2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    buttonUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    buttonDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    buttonLeft = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
    buttonRight = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    buttonB = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    buttonA = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    buttonX = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    buttonY = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        selectedAuton++;
        if(selectedAuton > 7) {
            selectedAuton = 0;
    }
 } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        selectedAuton--;
        if(selectedAuton < 0) {
            selectedAuton = 7;
    }
 }
 if (controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_R2)) {
   // buttonFlagB = 1;
 }
if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) || controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
    buttonFlagB++;
}
if (buttonFlagB % 2 == 0) {
    Claw.open();
} else {
    Claw.close();
}
if (r2) {
    lift.move(-127);
} else if (r1) {
    lift.move(127);
} else if (l1) {
    flipper.move(127);
} else if (l2) {
togglePiston.set_value(true);
} else {
    togglePiston.set_value(false);
    flipper.move(0);
    lift.move(0);
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}
 }

void opcontrol() {
 //pros::Task flipperTask(flipperControl);
    while (true) {
        // get left y and right y positions
   int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

driverControl();
        // delay to save resources
        pros::delay(10);
    }
}
