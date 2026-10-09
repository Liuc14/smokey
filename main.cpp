#include "main.h"
#include "lemlib/api.hpp"
#include "pros/rtos.hpp"
#include "pros/misc.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
// left motor group
pros::MotorGroup left_motor_group({-13, -14});
// right motor group
pros::MotorGroup right_motor_group({11, 12});

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, 
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 8 
);

// imu
pros::Imu imu(18);
//vertical odom tracker
pros::Rotation vertical_tracker(19);
//lift rotation sensor
pros::Rotation liftRotationSensor(-8);
//wall distance sensor for distance resets
pros::Distance distanceFront(5);
pros::Distance distanceRight(16);
pros::Distance distanceLeft(6);
pros::Distance distanceBack(17);
//distance sensor on claw for auto clamp
pros::Distance distanceClaw(21);
//side roller motor constructor
pros::Motor rollerIntake(21, pros::MotorGears::green);
pros::Motor frontIntake(-15, pros::MotorGear::blue);
//claw intake motor constructor
pros::Motor clawRoller(3, pros::MotorGears::blue);
//double lift motor constructor
pros::MotorGroup lift({-2, 7}, pros::MotorGears::green);
//double flipper motor constructor
pros::Motor flipper(4, pros::MotorGears::green);
//piston to open claw for grabbing cups and pins
pros::adi::DigitalOut clawPiston('B');
pros::adi::DigitalOut togglePiston('C');
pros::adi::DigitalOut rollerPiston('A');
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
                                              3, // derivative gain (kD)
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
float offsetFront = 4;
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
    flipper.move_absolute(380, 127);
    pros::delay(1250);
    flipper.move_absolute(0, -127);
}
void flipperControl() {
    while (true) {
         if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
    setFlipper();
 }
 pros::delay(10);
    }
}
void intake(int speed) {
    frontIntake.move(speed);
}

void clawIntake(int speed) {
    clawRoller.move(speed);
}


//Auto claw section
bool autoClaw = false;

void autoClamp() { 
    //opens claw and sets a condition to wait while the distance sensor isn't detecting an object in range, and clamps when object is officially in range. After logic, exits loop.
          Claw.open();
    pros::delay(1000);
    if (autoClaw == true) {
  
        while (distanceClaw.get_distance() > 100) {
            pros::delay(10);
        }
    Claw.close();
    autoClaw = false;
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


void CloseSplit() {
  setPose(56, 2, 270);
    resetBack();
    //resetRight();
    //eventual piston set for toggle
 togglePiston.set_value(true);
    //resetRight();
    //eventual piston set for toggle
    pros::delay(200);

    chassis.moveToPoint(45, chassis.getPose().y, 800);
    pros::delay(200);
    togglePiston.set_value(false);
    waitd;
    chassis.turnToPoint(47.5, 16, 850, {.forwards = false, .minSpeed = 4, .earlyExitRange = 10});
        liftTarget = 12500;
waitd;
    chassis.moveToPoint(47.5, 16, 600, {.forwards = false, .maxSpeed = 70});
    waitd;
    chassis.turnToHeading(180, 500);
    liftTarget = 0;
    pros::delay(600);
    Claw.open();
    pros::delay(100);
    waitd;


    
    resetLeft();
    chassis.moveToPoint(47.5, 1, 650, {.minSpeed = 10});
    waitd;
    Claw.close();
    liftTarget = 0;
    turnToPoint(56.5, 12.5, 450, {.forwards =false});
    waitd;
clawIntake(127);
    moveToPoint(56.5, 12.5, 800, {.forwards = false, .minSpeed = 10, .earlyExitRange = 2});
    waitd;

chassis.swingToHeading(270, DriveSide::LEFT, 650);
waitd;

    chassis.turnToPoint(52, -3, 500, {.minSpeed = 20, .earlyExitRange = 7});
    waitd;

    chassis.moveToPoint(52,-3, 1050, {.minSpeed = 20});
    waitd;
     liftTarget = 20000;

    chassis.turnToPoint(49, -22, 900, {.forwards = false, .minSpeed = 30});
    pros::delay(100);
   
    waitd;

    moveToPoint(49, -24, 600, {.forwards = false, .maxSpeed =65});
    waitd;

    turnToAngle(0, 600);
        liftTarget = 14000;
        clawIntake(0);
    pros::delay(300);
    Claw.open();
    waitd;
    resetRight();
    liftTarget = 20000;
    //liftTarget = 0;

    moveToPoint(53, -9, 1000, {.minSpeed = 20});
    pros::delay(150);
    liftTarget = -200;
    waitd;
    Claw.close();
    turnToPoint(58, -16.5, 500, {.forwards =false});
    waitd;
clawIntake(127);
    moveToPoint(58, -16.5, 800, {.forwards = false, .minSpeed =5});
    waitd;

chassis.swingToHeading(270, DriveSide::RIGHT, 800, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
waitd;
chassis.arcade(-80,0);
pros::delay(200);
chassis.arcade(0,0);
    chassis.turnToPoint(53, 0.5, 500, {.minSpeed = 20, .earlyExitRange = 10});
    waitd;

    chassis.moveToPoint(53,0.5, 1000);
    waitd;
    
    liftTarget = 20000;
    
    chassis.turnToPoint(49.5, 16, 850, {.forwards = false});
    waitd;
    clawIntake(0);

    chassis.moveToPoint(48, 16, 650, {.forwards = false, .maxSpeed = 70});
    waitd;

    liftTarget = 0;
    pros::delay(500);
    Claw.open();


}
void CloseElim() { 
    
  setPose(56, 2, 270);
    resetBack();
    //resetRight();
    //eventual piston set for toggle
 togglePiston.set_value(true);
    //resetRight();
    //eventual piston set for toggle
    pros::delay(200);

    chassis.moveToPoint(53.5, chassis.getPose().y, 950, {.minSpeed = 30, .earlyExitRange = 2});
    pros::delay(200);
    togglePiston.set_value(false);
    waitd;
    chassis.turnToPoint(52.5, 16, 850, {.forwards = false, .minSpeed = 4, .earlyExitRange = 10});
        liftTarget = 12500;
waitd;
    chassis.moveToPoint(52.5, 16, 600, {.forwards = false, .maxSpeed = 70});
    waitd;
    chassis.turnToHeading(180, 500);
    liftTarget = 0;
    pros::delay(600);
    Claw.open();
    pros::delay(100);
    waitd;


    
    resetLeft();
    liftTarget = -1250;
    moveToPoint(47.5, 0, 1000, {.minSpeed = 10});
    waitd;



    turnToPoint(33, 15, 500, {.forwards = false});
    waitd;

    clawIntake(127);
    moveToPoint(33, 15, 1400, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntil(18);
    Claw.close();
    waitd;

moveToPoint(32, 18, 500, {.forwards = false, .maxSpeed = 70});
    waitd;


    turnToPoint(51, 20, 1000, {.forwards = false});
     liftTarget = 17500;
    waitd;
    clawIntake(0);
   
   // lift.move_absolute(600, 600);
    moveToPoint(50, 21, 900, {.forwards = false, .maxSpeed = 70});
    waitd;
   // lift.move(-127);
     


    turnToAngle(270, 400);
    liftTarget = 0;
    pros::delay(200);
    liftTarget = 1250;
pros::delay(400);
Claw.open();
  waitd;
    resetRight();
    liftTarget = 0;

    moveToPoint(33, 20, 800);
    waitd;
liftTarget = -4000;
    
    turnToPoint(50, 43.5, 600, {.forwards = false});
    waitd;
    liftRotationSensor.reset_position();
    clawIntake(127);
liftTarget =0;
    moveToPoint(50, 43.5, 1100, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntil(22);
    Claw.close();
    waitd;

    moveToPoint(51, 44, 400, {.forwards = false});
    waitd;

    turnToPoint(56, 22, 800, {.forwards = false}); 
        liftTarget = 27500;
    waitd;

    clawIntake(0);
   
    moveToPoint(56, 22, 1200, {.forwards = false, .maxSpeed = 65});  
   // lift.move_absolute(1200, 600);

    waitd;
    liftTarget = 2000;
    pros::delay(900);
Claw.open();




    /*chassis.moveToPoint(48, 4, 650);
    waitd;
    Claw.close();
    liftTarget = -500;
    turnToPoint(54, 12.8, 400, {.forwards =false});
    waitd;
clawIntake(127);
    moveToPoint(54, 12.8, 700, {.forwards = false, .minSpeed = 10 , .earlyExitRange = 6});
    waitd;

    chassis.moveToPoint(62, 14.3, 350, {.forwards = false});
    waitd;
chassis.swingToHeading(270, DriveSide::LEFT, 150);
waitd;
chassis.arcade(-80, 8);
pros::delay(400);
chassis.arcade(0, 0);
chassis.turnToHeading(25, 1250, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 5, .earlyExitRange = 10});
waitd;

chassis.moveToPoint(57, 26.5, 700, {.minSpeed = 20, .earlyExitRange = 2});
waitd;

liftTarget = 2500;

chassis.turnToPoint(52.5, 17, 700, {.forwards =false});
waitd;

chassis.moveToPoint(52.5, 17, 800, {.forwards = false, .maxSpeed = 70});

waitd;
clawIntake(0);
liftTarget = -500;
pros::delay(300);
Claw.open();

chassis.moveToPoint(66, 27, 1000);
waitd;

chassis.turnToPoint(51, 36, 650, {.forwards =false});
waitd;
clawIntake(127);
chassis.moveToPoint(51, 36, 700, {.forwards = false, .minSpeed = 20, .earlyExitRange = 3});
chassis.waitUntil(5.5);
Claw.close();
waitd;


chassis.turnToPoint(50, 18, 1000, {.forwards = false, .direction = AngularDirection::CCW_COUNTERCLOCKWISE});
liftTarget = 5250;
waitd;

clawIntake(0);

chassis.moveToPoint(52, 18, 1400, {.forwards =false, .maxSpeed = 55});
waitd;

liftTarget = 0;
pros::delay(300);
Claw.open();
pros::delay(100);
*/
/*
chassis.moveToPoint(51, 28, 800, {.minSpeed = 15});
waitd;

chassis.turnToPoint(28,12, 500, {.forwards = false});
waitd;
liftTarget = 0;
clawIntake(127);
chassis.moveToPoint(28,12,900, {.forwards = false,  .maxSpeed = 90});
chassis.waitUntil(21);
Claw.close();
waitd;

chassis.turnToPoint(48, 15, 700, {.forwards =false});
liftTarget = 5000;
waitd;
chassis.moveToPoint(48, 15, 800, {.forwards = false, .maxSpeed = 70});
waitd;
liftTarget = 4000;




*/



/*
    chassis.turnToPoint(49, -1, 400, {.minSpeed = 20, .earlyExitRange = 10});
    waitd;

    chassis.moveToPoint(49,-1, 900, {.minSpeed = 10, .earlyExitRange = 2});
    waitd;


      liftTarget = 2750;

    moveToPoint(47.5, 15, 800, {.forwards =false, .maxSpeed = 70});
    waitd;

    turnToAngle(180, 500);
    liftTarget = 0;
    pros::delay(300);
    Claw.open();
    waitd;
    resetLeft();

    moveToPoint(48, 0, 1000, {.minSpeed = 10});
    waitd;



    turnToPoint(27, 13, 500, {.forwards = false});
    waitd;

    clawIntake(127);
    moveToPoint(27, 13, 1000, {.forwards = false, .maxSpeed = 110});
    chassis.waitUntil(20);
    Claw.close();
    waitd;

   // lift.move_absolute(100, 600);

    turnToPoint(48, 23, 700, {.forwards = false, .minSpeed = 5});
    liftTarget = 4000;
    waitd;
    clawIntake(0);
   // lift.move_absolute(600, 600);
    moveToPoint(46, 22, 900, {.forwards = false, .maxSpeed = 80});
    waitd;
   // lift.move(-127);
     


    turnToAngle(270, 400);
    liftTarget = 1750;
pros::delay(200);
Claw.open();
  waitd;
    resetRight();
    liftTarget = 0;

    moveToPoint(32, 20, 800);
    waitd;

    
    turnToPoint(45, 41, 500, {.forwards = false});
    waitd;
    clawIntake(127);

    moveToPoint(45, 41, 900, {.forwards = false, .maxSpeed = 110});
    chassis.waitUntil(16);
    Claw.close();
    waitd;

    turnToPoint(55, 22, 650, {.forwards = false}); 
    waitd;

    clawIntake(0);
    liftTarget = 4250;
   
    moveToPoint(53, 20, 900, {.forwards = false, .maxSpeed = 70});  
   // lift.move_absolute(1200, 600);

    waitd;
    liftTarget = 3000;
    pros::delay(800);
Claw.open();
      */
}
void FarSplit() {
 setPose(2, 56, 180);
    resetBack();
    //resetRight();
    //eventual piston set for toggle
    togglePiston.set_value(true);
pros::delay(200);
    chassis.moveToPoint(chassis.getPose().x, 51, 800, {.minSpeed = 30, .earlyExitRange = 1});
    pros::delay(200);
    togglePiston.set_value(false);
    waitd;
    chassis.turnToPoint(18, 52.5, 800, {.forwards = false, .minSpeed = 4, .earlyExitRange = 15});
        liftTarget = 15000;
waitd;
    chassis.moveToPoint(18, 52.5, 600, {.forwards = false, .maxSpeed = 70});
    waitd;
    chassis.turnToHeading(270, 500);
    liftTarget = 4000;
    pros::delay(600);
    Claw.open();
    pros::delay(100);
    waitd;


    
    resetRight();
    chassis.moveToPoint(4, 51, 650);
    waitd;
    Claw.close();
    liftTarget = -2000;
    turnToPoint(10.25, 56.5, 400, {.forwards =false});
    waitd;
clawIntake(127);
    moveToPoint(10.25, 56.5, 900, {.forwards = false, .minSpeed = 10});
    waitd;
chassis.swingToHeading(180, DriveSide::RIGHT, 650, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
waitd;
chassis.arcade(-80,0);
pros::delay(200);
chassis.arcade(0,0);

    chassis.turnToPoint(-8, 54, 400, {.minSpeed = 20, .earlyExitRange = 10});
    waitd;

    chassis.moveToPoint(-8,54, 1000, {.minSpeed = 15});
    waitd;


    chassis.turnToPoint(-24, 49, 925, {.forwards = false, .direction = AngularDirection::CCW_COUNTERCLOCKWISE});
        liftTarget = 22500;
    waitd;

    moveToPoint(-24, 49, 500, {.forwards = false, .maxSpeed =70});
    waitd;

    turnToAngle(90, 650);
        liftTarget = 14000;
        clawIntake(0);
    pros::delay(400);
    Claw.open();
    waitd;
    resetLeft();
    liftTarget = 21000;
    //liftTarget = 0;

    moveToPoint(-12, 50, 1000);
    pros::delay(150);
    liftTarget = -2000;
    waitd;
    Claw.close();
    turnToPoint(-19.5, 57.5, 550, {.forwards =false});
    waitd;
clawIntake(127);
    moveToPoint(-19.5, 57.5, 800, {.forwards = false});
    waitd;

chassis.swingToHeading(275, DriveSide::LEFT, 600, {.direction = AngularDirection::CW_CLOCKWISE, .maxSpeed = 110});
waitd;
chassis.arcade(-80,0);
pros::delay(200);
chassis.arcade(0,0);
    chassis.turnToPoint(2,53, 400, {.minSpeed = 20, .earlyExitRange = 10});
    waitd;

    chassis.moveToPoint(2,53, 1100);
    waitd;

    liftTarget = 25000;
    clawIntake(0);
    chassis.turnToPoint(20,49, 900, {.forwards = false, .direction = AngularDirection::CCW_COUNTERCLOCKWISE});
    waitd;

    chassis.moveToPoint(20,49, 1000, {.forwards = false, .maxSpeed = 60});
    waitd;

    liftTarget = 0;
    pros::delay(500);
   // Claw.open();


}
void FarElim() {
     setPose(2, 56, 180);
    resetBack();
   // liftTarget = -500;
    togglePiston.set_value(true);
    //Claw.close();
    
  //  clawIntake(127);
    //resetRight();
    //eventual piston set for toggle
pros::delay(200);
    chassis.moveToPoint(chassis.getPose().x, 53.5, 800, {.minSpeed = 30, .earlyExitRange = 1});
    pros::delay(200);
    togglePiston.set_value(false);
    waitd;
    chassis.turnToPoint(16, 52.5, 800, {.forwards = false, .minSpeed = 4, .earlyExitRange = 15});
        liftTarget = 12500;
waitd;
    chassis.moveToPoint(16, 52.5, 600, {.forwards = false, .maxSpeed = 70});
    waitd;
    clawIntake(0);
    chassis.turnToHeading(270, 500);
    liftTarget = 500;
    pros::delay(650);
    Claw.open();
    pros::delay(200);
    waitd;


    
    resetRight();
    liftTarget = 0;
    moveToPoint(4,48, 1000, {.minSpeed = 10});
    waitd;



    turnToPoint(21, 26, 500, {.forwards = false});
    waitd;

    clawIntake(127);
    moveToPoint(21, 26, 1100, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntil(15.5);
    Claw.close();
    waitd;

    moveToPoint(22,31, 500, {.forwards = false});
    waitd;

   // lift.move_absolute(100, 600);

    turnToPoint(20, 48, 1000, {.forwards = false});
     liftTarget = 20000;
    waitd;
    clawIntake(0);
   
   // lift.move_absolute(600, 600);
    moveToPoint(20, 49, 900, {.forwards = false, .maxSpeed = 70});
    waitd;
   // lift.move(-127);
     


    turnToAngle(180, 400);
    liftTarget = 0;
    pros::delay(200);
    liftTarget = 1500;
pros::delay(300);
Claw.open();
  waitd;
    resetLeft();
    liftTarget = 0;

    moveToPoint(25.5, 32, 800);
    waitd;
liftTarget = -1000;
    
    turnToPoint(40, 47, 550, {.forwards = false});
    waitd;
    liftRotationSensor.reset_position();
    clawIntake(127);
liftTarget =0;
    moveToPoint(40, 47, 1100, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntil(14);
    Claw.close();
    waitd;

    moveToPoint(39, 49, 400, {.forwards = false});
    waitd;
  liftTarget = 25000;
    turnToPoint(22, 48, 750, {.forwards = false}); 
    waitd;

    clawIntake(0);
   
    moveToPoint(22, 48, 1000, {.forwards = false, .maxSpeed = 50});  
   // lift.move_absolute(1200, 600);

    waitd;
    liftTarget = 10000;

}


void skillsMatchLoad() {

    turnToAngle(0, 800);
    lift.move(-127);
    turnToAngle(0, 400);
   resetRight();
   pros::delay(100);
moveToPoint(51, 61, 2250, { .maxSpeed = 60});

waitd;
Claw.close();
turnToAngle(270, 800);
waitd;
clawIntake(127);

moveToPoint(69, 61.5, 1000, {.forwards = false, .maxSpeed = 50});
waitd;

pros::delay(350);

moveToPoint(51,61, 1000);
waitd;


}
void Skills() {
      setPose(56, 2, 270);
    resetBack();
    //resetRight();
    //eventual piston set for toggle
 togglePiston.set_value(true);
    //resetRight();
    //eventual piston set for toggle
    pros::delay(200);

    chassis.moveToPoint(45, chassis.getPose().y, 1250);
    pros::delay(200);
    togglePiston.set_value(false);
    waitd;
    chassis.turnToPoint(47.5, 16, 850, {.forwards = false, .minSpeed = 4, .earlyExitRange = 10});
        liftTarget = 12500;
waitd;
    chassis.moveToPoint(47.5, 16, 600, {.forwards = false, .maxSpeed = 70});
    waitd;
    chassis.turnToHeading(180, 500);
    liftTarget = 0;
    pros::delay(600);
    Claw.open();
    pros::delay(100);
    waitd;


    
    resetLeft();
    liftTarget = -1250;
    moveToPoint(47.5, 0, 1000, {.minSpeed = 10});
    waitd;



    turnToPoint(27.5, 19, 500, {.forwards = false});
    waitd;

    clawIntake(127);
    moveToPoint(27.5, 19, 1400, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntil(19);
    Claw.close();
    waitd;

moveToPoint(24, 27, 900, {.forwards = false, .maxSpeed = 70});
    waitd;


    turnToPoint(50, 22, 1000, {.forwards = false});
     liftTarget = 22500;
    waitd;
    clawIntake(0);
   
   // lift.move_absolute(600, 600);
    moveToPoint(51, 21, 900, {.forwards = false, .maxSpeed = 70});
    waitd;
   // lift.move(-127);
     


    turnToAngle(270, 400);
    liftTarget = 0;
    pros::delay(200);
    liftTarget = 1250;
pros::delay(400);
Claw.open();
  waitd;
    resetRight();
    liftTarget = 0;

    moveToPoint(30, 25, 800);
    waitd;
liftTarget = -4000;
    
    turnToPoint(48, 46, 650, {.forwards = false});
    waitd;
    liftRotationSensor.reset_position();
    clawIntake(127);
liftTarget =0;
    moveToPoint(48, 46, 1100, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntil(20);
    Claw.close();
    waitd;

    moveToPoint(51, 44, 400, {.forwards = false});
    waitd;

    turnToPoint(51, 22, 800, {.forwards = false}); 
        liftTarget = 35000;
    waitd;

    clawIntake(0);
   
    moveToPoint(51, 22, 1200, {.forwards = false, .maxSpeed = 65});  
   // lift.move_absolute(1200, 600);

    waitd;
    liftTarget = 10000;
    pros::delay(900);
Claw.open();


/////////////////////////////////
       moveToPoint(50, 38, 1600, {.maxSpeed = 60});
       waitd;
       Claw.close();
           liftTarget = 0;
              // Claw.close();
    turnToAngle(0, 650);
    waitd;
    resetRight();
    moveToPoint(50, 62, 1250, {.maxSpeed = 90});
    waitd;

 clawIntake(127);
 Claw.open();
   turnToAngle(270, 700);
   waitd;
   resetRight();

    moveToPoint(69, 59, 800, {.forwards = false, .maxSpeed = 70});
    waitd;
    Claw.close();
 chassis.arcade(-60, 0);
    pros::delay(200);
    chassis.arcade(70, 0);
    pros::delay(200);
    chassis.arcade(0,0);

        chassis.arcade(-60, 0);
    pros::delay(200);
    chassis.arcade(70, 0);
    pros::delay(300);
    chassis.arcade(0,0);
          chassis.arcade(-60, 0);
    pros::delay(200);
    chassis.arcade(70, 0);
    pros::delay(300);
    chassis.arcade(0,0);
    moveToPoint(49, 59, 800);
    waitd;

    turnToPoint(50, 26, 1250, {.forwards = false});
    waitd;
    
    liftTarget=43000;

    moveToPoint(50, 26, 2000, {.forwards = false, .maxSpeed = 50});
    waitd;
    clawIntake(0);
    liftTarget = 30000;
    pros::delay(700);
Claw.open();

pros::delay(100);
intake(-60);
liftTarget = 50000;

pros::delay(500);
/////////




       moveToPoint(50, 38, 1600, {.maxSpeed = 60});
       waitd;
           liftTarget = 0;
               Claw.close();
    turnToAngle(0, 650);
    waitd;
    resetRight();


   

    moveToPoint(50, 62, 1250, {.maxSpeed = 90});
    waitd;

 clawIntake(127);
 Claw.open();
   turnToAngle(270, 700);
   waitd;
   resetRight();

    moveToPoint(69, 62, 800, {.forwards = false, .maxSpeed = 70});
    waitd;
        Claw.close();
   chassis.arcade(-60, 0);
    pros::delay(200);
    chassis.arcade(70, 0);
    pros::delay(200);
    chassis.arcade(0,0);

        chassis.arcade(-60, 0);
    pros::delay(200);
    chassis.arcade(70, 0);
    pros::delay(300);
    chassis.arcade(0,0);
          chassis.arcade(-60, 0);
    pros::delay(200);
    chassis.arcade(70, 0);
    pros::delay(300);
    chassis.arcade(0,0);

    moveToPoint(49, 60, 800);
    waitd;

    turnToPoint(50, 26, 1250, {.forwards = false});
    waitd;
    
    liftTarget=54000;

    moveToPoint(50, 26, 2800, {.forwards = false, .maxSpeed = 50});
    waitd;
    clawIntake(0);

    liftTarget = 45000;
    pros::delay(700);
    Claw.open();
    pros::delay(100);
    intake(-60);
    liftTarget = 60000;
    pros::delay(1000);
   // Claw.close();
/////////////////////





  moveToPoint(50, 38, 1600, {.maxSpeed = 60});
       waitd;
           liftTarget = 0;
               Claw.close();
    turnToAngle(0, 650);
    waitd;
    resetRight();


   

    moveToPoint(50, 62, 1250, {.maxSpeed = 90});
    waitd;

 clawIntake(127);
 Claw.open();
   turnToAngle(270, 700);
   waitd;
   resetRight();

    moveToPoint(69, 62, 800, {.forwards = false, .maxSpeed = 70});
    waitd;
        Claw.close();
    chassis.arcade(-60, 0);
    pros::delay(200);
    chassis.arcade(70, 0);
    pros::delay(200);
    chassis.arcade(0,0);

        chassis.arcade(-60, 0);
    pros::delay(200);
    chassis.arcade(70, 0);
    pros::delay(300);
    chassis.arcade(0,0);
          chassis.arcade(-60, 0);
    pros::delay(200);
    chassis.arcade(70, 0);
    pros::delay(300);
    chassis.arcade(0,0);

    moveToPoint(49, 60, 800);
    waitd;

    turnToPoint(48, 26, 1250, {.forwards = false});
    waitd;
    
    liftTarget=65000;
    pros::delay(1000);

    moveToPoint(48, 26, 2800, {.forwards = false, .maxSpeed = 50});
    waitd;
    clawIntake(0);

    liftTarget = 50000;
    pros::delay(800);
    Claw.open();
    pros::delay(400);
    intake(-60);
    liftTarget = 70000;
    pros::delay(1000);
   // Claw.close();
//////

    moveToPoint(50, 38, 1600, {.maxSpeed = 60});
       waitd;
       Claw.close();
           liftTarget = -5000;
              // Claw.close();
    turnToAngle(0, 650);
    waitd;
    resetRight();
    moveToPoint(48.5, 62, 1250, {.maxSpeed = 90});
    waitd;

 clawIntake(127);
 Claw.open();
   turnToAngle(270, 700);
   waitd;
   resetRight();

    moveToPoint(69, 60, 800, {.forwards = false, .maxSpeed = 50});
    pros::delay(400);
    Claw.close();
    waitd;
chassis.arcade(-30, 0);
pros::delay(200);
chassis.arcade(100, 0);
pros::delay(200);
chassis.arcade(0,0);
    liftRotationSensor.reset_position();
 
    moveToPoint(54, 59, 1600);
    waitd;
    liftTarget = 20000;

    turnToPoint(5,2,1000, {.forwards =false});
    waitd;

    liftTarget = 30000;
moveToPoint(5, 2, 3000, {.forwards = false, .maxSpeed = 70});
waitd;

liftTarget = 10000;
pros::delay(200);
Claw.open();


}

void closeAwp() {
    setPose(56, 2, 270);
    resetBack();
    //resetRight();
    togglePiston.set_value(true);//eventual piston set for toggle
pros::delay(150);
    chassis.moveToPoint(48.5, chassis.getPose().y, 800, {.minSpeed = 30, .earlyExitRange = 1});
    pros::delay(200);
    togglePiston.set_value(false);
    waitd;
    chassis.turnToPoint(47.25, 16, 800, {.forwards = false, .minSpeed = 4, .earlyExitRange = 15});
        liftTarget = 1750;
waitd;
    chassis.moveToPoint(47.25, 16, 600, {.forwards = false, .maxSpeed = 75});
    waitd;
    chassis.turnToHeading(180, 500);
    liftTarget = 500;
    pros::delay(550);
    Claw.open();
    pros::delay(100);
    waitd;


    
    resetLeft();
    chassis.moveToPoint(48.5, 4, 650, {.minSpeed = 40});
    waitd;
    Claw.close();
    liftTarget = -1000;
    turnToPoint(54, 12.5, 400, {.forwards =false});
    waitd;
clawIntake(127);
liftRotationSensor.reset_position();
    moveToPoint(54, 12.5, 700, {.forwards = false, .minSpeed = 10 , .earlyExitRange = 6});
    waitd;

    chassis.moveToPoint(62, 13.5, 350, {.forwards = false});
    waitd;
chassis.arcade(-80, 7);
pros::delay(100);
chassis.arcade(0,0);
    chassis.turnToPoint(48, -8, 400, {.minSpeed = 20, .earlyExitRange = 10});
    waitd;

    chassis.moveToPoint(48,-8, 900, {.minSpeed = 15, .earlyExitRange = 6});
    waitd;


    chassis.turnToPoint(58, -25, 600, {.forwards = false, . minSpeed = 5, .earlyExitRange = 4});
        liftTarget = 3250;
    waitd;

    moveToPoint(52, -25, 500, {.forwards = false, .maxSpeed =80, .minSpeed = 40});
    waitd;
        liftTarget = 0;

    turnToAngle(0, 400);
    
        clawIntake(0);
    pros::delay(350);
    Claw.open();
    waitd;
    resetRight();
    //liftTarget = 0;

    moveToPoint(50, -7, 650);
    waitd;

    turnToPoint(24.75, 15, 700, {.forwards = false});
    waitd;
    clawIntake(127);
    moveToPoint(24.75, 15, 1050, {.forwards = false, .minSpeed =2, .earlyExitRange = 3});
    chassis.waitUntil(18);
    Claw.close();
    waitd;

    liftTarget = 2500;

    moveToPoint(30.5, 48, 1000, {.forwards = false, .maxSpeed = 90});
    waitd;
  liftTarget = 1000;
turnToAngle(180, 400);
clawIntake(0);
  
    pros::delay(250);
    Claw.open();
    waitd;

    resetLeft();

    moveToPoint(28, 28, 800, {.minSpeed = 20});
    waitd;
    clawIntake(127);

    chassis.turnToPoint(45, 46, 650, {.forwards = false});
    waitd;

    chassis.moveToPoint(45, 46, 800, {.forwards = false});
    chassis.waitUntil(18);
    Claw.close();
    waitd;
liftTarget = 3250;
    chassis.turnToPoint(57, 24, 600, {.forwards = false, .minSpeed =2, .earlyExitRange = 4});
    waitd;
  

    chassis.moveToPoint(58, 26, 800, {.forwards = false});
    pros::delay(500);
    liftTarget = 0;
    waitd;
    

    

    /*

chassis.turnToHeading(10, 850, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 10, .earlyExitRange = 10});
waitd;

chassis.moveToPoint(56, 28, 900);
waitd;

liftTarget = 2500;

chassis.turnToPoint(51, 22, 500, {.forwards =false, .minSpeed = 2, .earlyExitRange = 8});
waitd;

chassis.moveToPoint(51, 22, 600, {.forwards = false});
waitd;

liftTarget = 1000;
Claw.open();




    //chassis.turnToPoint(56, 30, 500, {.minSpeed = 20, .earlyExitRange = 10});
    //waitd;

   // chassis.moveToPoint(56, 30, 700);
    //waitd;
//
/*liftTarget = 2250;
chassis.moveToPoint(48, 0, 1000, {.minSpeed = 5, .earlyExitRange = 4});
waitd;

chassis.moveToPoint(48, 24, 800, {.forwards = false});
waitd;
liftTarget = 0;
chassis.turnToHeading(180, 600);

    ///

   turnToPoint(56, 30, 800, {.forwards = false});
    waitd;

    moveToPoint(56, 30, 800, {.forwards = false});
    waitd;
    
liftTarget = 2500;

    clawIntake(0);

    turnToPoint(48, 20, 650, {.forwards = false, .minSpeed = 2, .earlyExitRange = 10});
    waitd;
    moveToPoint(48, 22, 700, {.forwards = false});
    waitd;
    liftTarget = 1000;
    pros::delay(300);
    Claw.open();
    pros::delay(100);
    chassis.swingToHeading(90, DriveSide::RIGHT, 850, {.direction = AngularDirection::CW_CLOCKWISE});
    waitd;
    resetLeft();

    turnToPoint(49, 42, 600, {.forwards = false});
    waitd;
clawIntake(127);
    moveToPoint(49, 42, 800, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntil(4);
    Claw.close();
    waitd;
    turnToPoint(47, 24, 800, {.forwards =false, .minSpeed = 5, .earlyExitRange = 15});
    clawIntake(0);
    waitd;

    liftTarget = 3000;
    moveToPoint(47, 24, 800, {.forwards = false, .maxSpeed = 80});
    waitd;
  
      liftTarget = 1000;
    pros::delay(200);
    Claw.open();
    pros::delay(100);



    moveToPoint(48, 30, 800);
    waitd;

    turnToAngle(0, 600);
    resetRight();

    turnToPoint(24, 24, 600, {.forwards = false});
    waitd;
clawIntake(127);
    moveToPoint(24, 24, 800, {.forwards = false});
    chassis.waitUntil(16);
    Claw.close();
    waitd;


*/




    

}
void pidTest() {}
void utilTest() {liftTarget = 10000;
pros::delay(2000);

liftTarget = 2000;}
//string for the names of our autonomous list, used to display which auton is selected for both controller and brain selector.
const char* autonomousNames[] = {
    "CloseSplit",
    "CloseElim ",
    "FarSplit  ",
    "FarElim   ",
    "Skills    ",
    "closeAwp  ",
    "utilTest  ",
    "None      "
};

//int determines which auton is ran.
int selectedAuton =2;

//case function which runs different autonomous routines based off the value of selectedAuton
void chooseAuton() {
switch(selectedAuton) {
    case 0:
        CloseSplit();
        break;
    case 1:
        CloseElim();
        break;
    case 2:
       FarSplit();
        break;
    case 3:
        FarElim();
        break;
    case 4:
        Skills();
        break;
    case 5:
       closeAwp();
        break;
    case 6:
    utilTest();
        break;
    case 7:
       // selectedAuton = 7;
        break;
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
    Claw.open();
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
      
     //   pros::Task autoClampTask(autoClamp);
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
    buttonFlagB = 1;
 }


 if(r1) {
  lift.move(127);
 } else if (r2) {
    lift.move(-127);
 } else if (buttonDown) {
        clawIndexBool = false;
    clawIntake(0);
     Claw.open();
 } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
// intake(127);
togglePiston.set_value(true);
 }
 else if (buttonRight) {
        rollerIntake.move(127);
        rollerPiston.set_value(true);
 } else if (buttonB == true) {
    clawIntake(127);
 } else if (buttonY) {
    Claw.open();
    clawIntake(127);
 } else if (l2) {
 // intake(-127);
 rollerPiston.set_value(true);
 } else {
    togglePiston.set_value(false);
    rollerIntake.move(0);
    Claw.close();
    lift.move(0);
    intake(0);
    rollerPiston.set_value(false);
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
 }
if(controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_B)) {
    clawIntake(0); }
     if(controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_Y)) {
    clawIntake(127);
 }

 }

void opcontrol() {
 pros::Task flipperTask(flipperControl);
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
