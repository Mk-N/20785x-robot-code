#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftMotor = motor(PORT15, ratio18_1, false);
motor RightMotor = motor(PORT20, ratio18_1, true);
motor LeftMotor2 = motor(PORT14, ratio18_1, false);
motor RightMotor2 = motor(PORT19, ratio18_1, true);
motor IntakeMotorA = motor(PORT9, ratio18_1, true);
motor IntakeMotorB = motor(PORT10, ratio18_1, false);
motor_group Intake = motor_group(IntakeMotorA, IntakeMotorB);
digital_out Indexer = digital_out(Brain.ThreeWirePort.B);
digital_out Expansion = digital_out(Brain.ThreeWirePort.A);
motor FlywheelMotorA = motor(PORT5, ratio6_1, true);
motor FlywheelMotorB = motor(PORT6, ratio6_1, false);
motor_group Flywheel = motor_group(FlywheelMotorA, FlywheelMotorB);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}