/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LeftMotor            motor         15              
// RightMotor           motor         20              
// LeftMotor2           motor         14              
// RightMotor2          motor         19              
// Intake               motor_group   9, 10           
// Indexer              digital_out   B               
// Expansion            digital_out   A               
// Flywheel             motor_group   5, 6            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath> //std::abs

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void index() {
  for (int i = 0; i < 3; i++){
    Indexer.set(false); 
    wait(150, msec);
    Indexer.set(true);
    wait(150, msec);

  }
}
void expand() {
  Expansion.set(false);
  wait(200,msec);
  Expansion.set(true);
  wait(200,msec);
  Expansion.set(false);
  wait(200,msec);
  Expansion.set(true);
  wait(200,msec);
  Expansion.set(false);
  wait(200,msec);
  Expansion.set(true);
  wait(200,msec);

}
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Intake.setVelocity(100, percent);
  Intake.setMaxTorque(100, percent);
  Indexer.set(true);
  Flywheel.setVelocity(60, percent);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

//Settings
float kP = 0.615;
float kI = 0.0;
float kD = 0.07;
float turnkP = 0.4;
float turnkI = 0.0;
float turnkD = 0.0;
int maxTurnIntegral = 300; // These cap the integrals
int maxIntegral = 300;
int integralBound = 3; //If error is outside the bounds, then apply the integral. This is a buffer with +-integralBound degrees

bool resetLateral = false;
bool resetTurn = false;

//Autonomous Settings
double desiredValue = 0;
double desiredTurnValue = 0;

double error; //SensorValue - DesiredValue : Position
double prevError = 0; //Position 20 miliseconds ago
double derivative; // error - prevError : Speed
double totalError = 0; //totalError = totalError + error

double turnError; //SensorValue - DesiredValue : Position
double turnPrevError = 0; //Position 20 miliseconds ago
double turnDerivative; // error - prevError : Speed
double turnTotalError = 0; //totalError = totalError + error

bool slowDrive = false;
bool mediumDrive = false;

bool resetDriveSensors = false;

//Variables modified for use
bool enableDrivePID = true;

//Pasted from a C++ resource
double signnum_c(double x) {
  if (x > 0.0) return 1.0;
  if (x < 0.0) return -1.0;
  return x;
}

int drivePID(){
  
  while(enableDrivePID){

    if (resetDriveSensors) {
      resetDriveSensors = false;
      LeftMotor.setPosition(0,degrees);
      RightMotor.setPosition(0,degrees);
    }


    if (resetLateral) {
      resetLateral = false;
      resetDriveSensors = false;
      LeftMotor.setPosition(0,degrees);
      RightMotor.setPosition(0,degrees);
      desiredValue = 0;
    }

    if (resetTurn) {
      resetTurn = false;
      resetDriveSensors = false;
      LeftMotor.setPosition(0,degrees);
      RightMotor.setPosition(0,degrees);
      desiredTurnValue = 0;
    }

    //Get the position of both motors
    int leftMotorPosition = LeftMotor.position(degrees);
    int rightMotorPosition = RightMotor.position(degrees);

    ///////////////////////////////////////////
    //Lateral movement PID
    /////////////////////////////////////////////////////////////////////
    //Get average of the two motors
    int averagePosition = (leftMotorPosition + rightMotorPosition)/2;

    //Potential
    error = desiredValue - averagePosition;

    //Derivative
    derivative = error - prevError;

    //Integral
    if(fabs(error) < integralBound){
    totalError+=error; 
    }  else {
    totalError = 0; 
    }
    //totalError += error;

    //This would cap the integral
    totalError = fabs(totalError) > maxIntegral ? signnum_c(totalError) * maxIntegral : totalError;

    double lateralMotorPower = error * kP + derivative * kD + totalError * kI;
    /////////////////////////////////////////////////////////////////////


    ///////////////////////////////////////////
    //Turning movement PID
    /////////////////////////////////////////////////////////////////////
    //Get average of the two motors
    int turnDifference = leftMotorPosition - rightMotorPosition;

    //Potential
    turnError = desiredTurnValue - turnDifference;

    //Derivative
    turnDerivative = turnError - turnPrevError;

    
    //Integral
    if(fabs(turnError) < integralBound){
    turnTotalError+=turnError; 
    }  else {
    turnTotalError = 0; 
    }
    //turnTotalError += turnError;

    //This would cap the integral
    turnTotalError = fabs(turnTotalError) > maxTurnIntegral ? signnum_c(turnTotalError) * maxTurnIntegral : turnTotalError;

    double turnMotorPower = turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI;
    /////////////////////////////////////////////////////////////////////

    if (slowDrive == false) { 
      if (mediumDrive == false) { 
        LeftMotor.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
        LeftMotor2.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
        RightMotor.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);
        RightMotor2.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    }
    else if (mediumDrive == true) {
    LeftMotor.spin(forward, lateralMotorPower/4 + turnMotorPower, voltageUnits::volt);
    LeftMotor2.spin(forward, lateralMotorPower/4 + turnMotorPower, voltageUnits::volt);
    RightMotor.spin(forward, lateralMotorPower/4 - turnMotorPower, voltageUnits::volt);
    RightMotor2.spin(forward, lateralMotorPower/4 - turnMotorPower, voltageUnits::volt);
    }
    
    LeftMotor.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    LeftMotor2.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    RightMotor.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    RightMotor2.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    }
    else if (slowDrive == true) {
    LeftMotor.spin(forward, lateralMotorPower/10 + turnMotorPower, voltageUnits::volt);
    LeftMotor2.spin(forward, lateralMotorPower/10 + turnMotorPower, voltageUnits::volt);
    RightMotor.spin(forward, lateralMotorPower/10 - turnMotorPower, voltageUnits::volt);
    RightMotor2.spin(forward, lateralMotorPower/10 - turnMotorPower, voltageUnits::volt);
    }
    
    

    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);

  }

  return 1;
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {

  vex::task auton(drivePID);
  enableDrivePID = true;
resetDriveSensors = true;
slowDrive = true;   
wait(2,sec);
desiredValue = -95;
wait(1,sec);
resetLateral = true;
Intake.spinFor(reverse, 350, degrees);
wait(0.5,sec);
desiredValue = 300;
wait(1, sec);
resetDriveSensors = true;
desiredValue = 0;
Intake.spin(reverse);
desiredTurnValue = -424.5;
waitUntil(error<30);
Brain.Screen.print(error);
wait(1, sec);
Brain.Screen.clearScreen();
resetTurn = true;
wait(0.7, sec);
desiredValue = -400;
wait(1.5, sec);
Intake.stop();
desiredValue = -655;//// GO LEWS COMPLE6TE
wait(1, sec);
resetLateral = true;
Intake.spinFor(reverse, 350, degrees);
desiredValue = 200;
Intake.spin(reverse);
wait(0.8,sec);
desiredTurnValue = 465; // TURN LESS COMP[PLEYTE]
waitUntil(error<30);
Brain.Screen.print(error);
wait(1, sec);
Brain.Screen.clearScreen();
resetTurn = true;
Flywheel.spin(reverse);
Intake.spin(reverse);

Flywheel.spin(reverse);
desiredValue = 950;
Intake.stop();
wait(1.8, sec);
resetLateral = true;
index();///////// FIRSTSHOT
wait(1, sec);
Flywheel.stop();
desiredValue = -520;
wait(1.5, sec);
resetLateral = true;

mediumDrive = true;

desiredTurnValue = 640;
Flywheel.spin(reverse);
waitUntil(error<30);
Brain.Screen.print(error);
wait(1, sec);
Brain.Screen.clearScreen();
resetTurn = true;
Intake.spin(reverse);
Flywheel.setVelocity(65,percent);
Flywheel.spin(reverse);
////////slowdrivec
slowDrive= true;
desiredValue = -700;
wait(2, sec);
desiredValue = -1050;
wait(2, sec);
resetLateral = true;
desiredTurnValue=-60;
wait (1,sec);
resetTurn = true;
desiredValue = -250;
wait(2, sec);
resetLateral = true;
desiredTurnValue = -400;
waitUntil(error<30);
Brain.Screen.print(error);
wait(1, sec);
Brain.Screen.clearScreen();
resetTurn = true;
wait(1.5, sec);


index();////// 2ND SHOT

Flywheel.stop();
desiredTurnValue = 300;
waitUntil(error<30);
Brain.Screen.print(error);
wait(2, sec);
Brain.Screen.clearScreen();
resetTurn = true;
desiredValue= -600;
wait(3, sec);
resetLateral = true;
wait(1,sec);
Intake.spin(reverse);
desiredValue= -630;
wait(2, sec);
resetLateral = true;
Intake.spin(reverse);
wait(1, sec);
desiredTurnValue= 220;
Intake.stop();
waitUntil(error<30);
Brain.Screen.print(error);
wait(1, sec);
Brain.Screen.clearScreen();
resetTurn = true;
wait(1,sec);
desiredValue = -100;
wait(1, sec);
resetLateral = true;
Intake.spinFor(reverse,150, degrees);
wait(1,sec);
Flywheel.setVelocity(70,percent);
Flywheel.spin(reverse);
desiredValue= 500;
wait(1.5, sec);
resetLateral= true;
wait(1, sec);
desiredTurnValue = 140;
waitUntil(error<30);
Brain.Screen.print(error);
wait(1, sec);
Brain.Screen.clearScreen();
resetTurn= true;
wait(1,sec);
index();
Flywheel.stop();
wait(1,sec);
desiredTurnValue = -450;
waitUntil(error<30);
Brain.Screen.print(error);
wait(1, sec);
Brain.Screen.clearScreen();
resetTurn = true;
wait (1, sec);
desiredValue = -500;
wait(1, sec);
resetLateral = true;
wait(0.5, sec);
Intake.spinFor(reverse,150,degrees);
desiredValue = 200;
wait(1,sec);
resetLateral = true;
desiredTurnValue = -450;
waitUntil(error<30);
Brain.Screen.print(error);
wait(1, sec);
Brain.Screen.clearScreen();
resetTurn = true;
desiredValue= -300;
wait(1,sec);
resetLateral = true;
expand();










































  
  

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {

  enableDrivePID = false;

  ///////////////////////////
  //Settings
  ///////////////////////////////////////////////////////////////////////////

  //Drivetrain
  double turnImportance = 0.5;

  while (1) {

    ///////////////////////////
    //Driver Control
    ///////////////////////////////////////////////////////////////////////////
    double turnVal = Controller1.Axis1.position(percent);
    double forwardVal = Controller1.Axis3.position(percent);

    double turnVolts = turnVal * 0.12;
    double forwardVolts = forwardVal * 0.12 * (1 - (std::abs(turnVolts)/12.0) * turnImportance);

    //0 - 12 = -12
    //0 + 12 = 12(due to cap)


    LeftMotor.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
    LeftMotor2.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
    RightMotor.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);
    RightMotor2.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);
    ///////////////////////////////////////////////////////////////////////////


 



    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
