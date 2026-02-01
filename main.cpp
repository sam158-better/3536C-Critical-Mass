#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
//#include "pros/vision.hpp"
//#include "autons.h"
//#include "constants.h"
//#pragma once


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Controller partner(pros::E_CONTROLLER_PARTNER);

// Motor Groups
pros::MotorGroup leftMotors({-4, -3, -10},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({18, 19, 9}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

//motors
pros::Motor front(1, pros::MotorGearset::blue); //front motor
pros::Motor scoring(2, pros::MotorGearset::blue); //scoring motor

//Sensors
pros::Vision vision_sensor (11); // Vision sensor on port 5
pros::Distance my_distance(12); // Distance sensor on port 6
pros::Optical my_optical(9); // Optical sensor on port 7
pros::Imu imu(5);

//Pnuematics
pros::adi::DigitalOut matchload('F', true);
pros::adi::DigitalOut wing('H', false); //false is extended, true is retracted
pros::adi::DigitalOut middle('A', true); //false is retracted, true is extended

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
//pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-7);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -0.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(7, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            17, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            127 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(3, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             19, // derivative gain (kD)
                                             0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              127 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

//void initAutonSelector();
//void runAuton();



//------------------------------------INTAKE/SCORING FUNCTIONS--------------------------------------------//

//constants

void intake(){
    front.move_velocity(-2000);
    scoring.move_velocity(0);
    middle.set_value(false); //extend middle pnuematic
}
void topScore(){
    front.move_velocity(-2000);
    scoring.move_velocity(-2000);
    middle.set_value(false); //retract middle pnuematic
}

void middleScore(){
    front.move_velocity(-2000);
    scoring.move_velocity(2000);
    middle.set_value(true); //retract middle pnuematic
}

void lowScore(){
    front.move_velocity(2000);
    scoring.move_velocity(2000);
    middle.set_value(false); //retract middle pnuematic
}
void stopEverything(){
    front.move_velocity(0);
    scoring.move_velocity(0);
    middle.set_value(false); //retract middle pnuematic
}




//------------------------------------AUTONOMOUS FUNCTIONS--------------------------------------------//

void skillsAuto() {
    chassis.setPose(0,0,0);

      intake();
    //move to the 4 balls
    chassis.moveToPose(19.2, 19.7, 52.3, 1000);
    //move in between the goal and matchloader
    chassis.moveToPoint(54.3, 1.0, 1000, {.maxSpeed = 80});
    //put down matchloader
    pros::delay(600);
    matchload.set_value(false);
    chassis.turnToHeading(180, 1000);
    //move into long goal and score
    chassis.moveToPoint(46.5, 15.9, 2500, {.forwards = false});
    pros::delay(500);
    topScore();
    pros::delay(2000);
    //switch back to intaking
    intake();
    //matchload
    chassis.moveToPoint(48, -18.5, 2000, {.maxSpeed = 60});
    chassis.moveToPoint(48, -17.5, 500, {.maxSpeed = 60});
    chassis.moveToPoint(48, -18.5, 1000, {.maxSpeed = 60});

   // chassis.moveToPoint(48.5, 15.9, 2000, {.forwards = false});
    //topScore();

    //move beside long goal and go to other side of field
    chassis.moveToPoint(59.5, 10.9, 1000, {.forwards = false});
    pros::delay(1000);
    chassis.moveToPoint(61.5, 95.9, 2000, {.forwards = false});

    //line up with goal and score
    chassis.moveToPoint(51, 95, 1000, {.maxSpeed = 60});
    chassis.turnToHeading(0, 500);
    chassis.moveToPose(50, 67, 0, 4500, {.forwards = false});

    //intake motions to prevent jamming
    pros::delay(1000);
    lowScore();
    pros::delay(300);
    topScore();
    pros::delay(3500);
    //switch back to intaking
    intake();

    //move to matchloader
    chassis.moveToPoint(51.2, 108.7, 2000, {.maxSpeed = 60});
    pros::delay(3100);

    //score on long goal again
    chassis.moveToPose(52, 70, 0, 4000, {.forwards = false});
    pros::delay(1000);
    lowScore();
    pros::delay(300);
    topScore();
    pros::delay(3000);
    //chassis.moveToPoint(52, 86, 500, {.maxSpeed = 60});

//do the following command to set up on red left side to just test the far part of skills auton
    //chassis.setPose({52, 70 , 0});

    //comment everything above the command if you want to do this^^^^^^



    intake();

   /* //intake balls and middle score
    intake();
    matchload.set_value(true);
    chassis.moveToPoint(47, 85, 1000, {.maxSpeed = 60});
    chassis.moveToPoint(21, 61, 2000, {.maxSpeed = 60});
    pros::delay(1500);
    matchload.set_value(false);
    chassis.turnToHeading(45, 500);
    chassis.moveToPose(12, 46.5, 45, 3000, {.forwards = false});
    pros::delay(800);
    lowScore();
    pros::delay(500);
    middleScore();
    pros::delay(3000);
    intake();
*/
    //move to other side of field
    chassis.moveToPoint(21, 65, 1000, {.maxSpeed = 60});
    chassis.turnToHeading(90, 500);



    chassis.moveToPoint(-21, 65, 1500, {.maxSpeed = 60});

    matchload.set_value(true);
    pros::delay(2500);
    matchload.set_value(false);


    //-------------now on to the left side of the field----------------//

    //line up with goal and score
    chassis.moveToPoint(-52, 87, 2000, {.maxSpeed = 60});
    chassis.turnToHeading(0, 500);
    chassis.moveToPose(-50, 65, 0, 4500, {.forwards = false});

    
    //intake motions to prevent jamming
    pros::delay(1000);
    lowScore();
    pros::delay(300);
    topScore();
    pros::delay(3500);
    //switch back to intaking
    intake();

    //move to matchloader
    chassis.moveToPoint(-49.2, 108.7, 2000, {.maxSpeed = 60});
    pros::delay(3100);

    //move to side of long goal and go down field
    chassis.moveToPoint(-60.5, 95.9, 2000, {.forwards = false});
    pros::delay(1000);
    matchload.set_value(true);
    chassis.moveToPoint(-60.5, 7.9, 2000, {.forwards = false});

    //move in between the goal and matchloader
    chassis.moveToPoint(-49.3, -1.0, 1000, {.forwards = false});
    chassis.turnToHeading(180, 1000);

    //move into long goal and score
    chassis.moveToPoint(-46.5, 15.9, 3500, {.forwards = false});
    pros::delay(500);
    topScore();
    pros::delay(2000);
    //switch back to intaking
    intake();

    matchload.set_value(false);

    //matchload
    chassis.moveToPoint(-48, -25.5, 2000, {.maxSpeed = 60});
    chassis.moveToPoint(-48, -15.5, 500, {.maxSpeed = 60});
    chassis.moveToPoint(-48, -25.5, 1000, {.maxSpeed = 60});

    //move into long goal and score
    chassis.moveToPoint(-48.5, 15.9, 2500, {.forwards = false});
    pros::delay(500);
    topScore();
    pros::delay(2000);

    //move in between the goal and matchloader
    chassis.moveToPoint(-50.3, 1.0, 1000, {.maxSpeed = 80});

    intake();

    matchload.set_value(true);
    //move by parking zone
    chassis.moveToPose(-19.3, -15.0, 90, 1000, {.maxSpeed = 80});

   //move into parking zone
    chassis.moveToPose(1, -30.0, 90, 1000, {.maxSpeed = 120});

}




void right7ball() {
    
chassis.setPose(0,0,0);

intake();
chassis.moveToPose(11.3, 20.5, 29.3, 2000, {.maxSpeed = 80});
//chassis.moveToPose(40, 40, 90, 2500, {.maxSpeed = 60});
pros::delay(1000);
//matchload.set_value(false);
//chassis.moveToPose(7, 15, 29.3, 1500, {.forwards = false});
chassis.moveToPoint(35.2, 5.4, 1500, {.maxSpeed = 80});
chassis.turnToHeading(180, 500);
chassis.moveToPoint(38 , 23, 2500, {.forwards = false});
pros::delay(500);
lowScore();
pros::delay(300);
topScore();
pros::delay(2500);
intake();
matchload.set_value(false);
chassis.moveToPoint(37.2, -14.5, 2000);
chassis.moveToPoint(35, 23, 2500, {.forwards = false});
pros::delay(500);
topScore();
chassis.moveToPoint(35.2, 5.4, 1000, {.maxSpeed = 80});
chassis.moveToPoint(25.8, 11.7, 1000, {.forwards = false});
chassis.turnToHeading(180, 500);
chassis.moveToPoint(25.1, 33.0, 1000, {.forwards = false});

}




void left7ball() {

intake();
chassis.moveToPose(-11.3, 20.5, 29.3, 2000, {.maxSpeed = 80});
//chassis.moveToPose(40, 40, 90, 2500, {.maxSpeed = 60});
pros::delay(1000);
//matchload.set_value(false);
//chassis.moveToPose(7, 15, 29.3, 1500, {.forwards = false});
chassis.moveToPoint(-35.2, 5.4, 1500, {.maxSpeed = 80});
chassis.turnToHeading(180, 500);
chassis.moveToPoint(-38 , 23, 2500, {.forwards = false});
pros::delay(500);
lowScore();
pros::delay(300);
topScore();
pros::delay(2500);
intake();
matchload.set_value(false);
chassis.moveToPoint(-37.2, -14.5, 2000);
chassis.moveToPoint(-35, 23, 2500, {.forwards = false});
pros::delay(500);
topScore();
chassis.moveToPoint(-35.2, 5.4, 1000, {.maxSpeed = 80});
chassis.moveToPoint(-25.8, 11.7, 1000, {.forwards = false});
chassis.turnToHeading(180, 500);
chassis.moveToPoint(-25.1, 33.0, 1000, {.forwards = false});

}




void SWP() {
    
chassis.setPose(0,0,90);

intake();

chassis.moveToPoint(54.3, 0.0, 1000, {.maxSpeed = 80});

chassis.turnToHeading(180, 500);

matchload.set_value(false);

chassis.moveToPoint(48, -18.5, 2000, {.maxSpeed = 60});

chassis.moveToPoint(48, 16, 2500, {.maxSpeed = 60});

topScore();
pros::delay(2400);
intake();

matchload.set_value(true);

chassis.swingToHeading(-90, DriveSide::LEFT, 500);

chassis.moveToPoint(20, 20, 1000, {.maxSpeed = 60});

chassis.moveToPoint(-25, 20, 1500, {.maxSpeed = 60});

matchload.set_value(false);

chassis.turnToHeading(-135, 500);

chassis.moveToPoint(-30, 35, 2100, {.forwards = false});

lowScore();
pros::delay(300);
middleScore();
pros::delay(2000);
intake();

chassis.moveToPoint(-54.3, 0.0, 1000, {.maxSpeed = 80});

chassis.moveToPoint(-54.3, 17.0, 1000, {.forwards = false});

topScore();
pros::delay(3000);

}

void right4ball() {

}

void left4ball() {

}

//-------------------------------------------end of autonomous functions-----------------------------------------//




void initialize() {

    //initAutonSelector();



    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
//ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {

    bool forwards = true;
    int maxSpeed = 80;

// Run the desired auton defined in autons.cpp here 
    //runAuton();
 
 /*   //lateral pid example
    chassis.setPose({0,0,0}); // set starting position (x, y, theta)
    chassis.moveToPose(0, 20, 0,  100000);
*/
  //angular pid example
    //chassis.setPose({0,0,0}); // set starting position (x, y, theta)
    //chassis.turnToHeading(90, 100000);

    skillsAuto();


}


   


/**
 * Runs in driver control
 */
void opcontrol() {
/*pros::vision_signature_s_t RED_SIG = 
    pros::Vision::signature_from_utility(
        1,       // ID of the signature (1-7)
        8973,    // U_min
        11143,   // U_max
        10058,   // U_mean
        -2119,   // V_min
        -1053,   // V_max
        -1586,   // V_mean
        5.4,     // Range
        0        // Type
    );
*/
    // controller
    // loop to continuously update motors
    bool wingstate = false;
    bool matchloadstate = false;
    //int object_count = vision_sensor.get_object_count();


    while (true) {

    //vision_sensor.set_signature(1, &RED_SIG);



        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);


        //intake control
        if(controller.get_digital(DIGITAL_R2)){
            intake();
        }
        //top score control
        else if(controller.get_digital(DIGITAL_R1)){
            topScore();
        }
       /* //extend wing
        else if(controller.get_digital(DIGITAL_L1)){
            wing.set_value(true); //extend wings
        }
        //retract wing
        else if(controller.get_digital(DIGITAL_L2)){
            wing.set_value(false); //retract wings
        }
        */
        //toggle matchloader
        else if(controller.get_digital(DIGITAL_Y)){
            if(matchloadstate == false){
                matchloadstate = true;
                matchload.set_value(true);
                pros::delay(500); //debounce
            }
            else{
                matchloadstate = false;
                matchload.set_value(false);
                pros::delay(500); //debounce
            } 
        }
        //middle score control
        else if(controller.get_digital(DIGITAL_L1)){
            middleScore();
        }
        //low score control
        else if(controller.get_digital(DIGITAL_L2)){
            lowScore();
        }
        else{
            stopEverything();
        }
        
        
        
        
        // delay to save resources
        pros::delay(10);
        
    }
} 