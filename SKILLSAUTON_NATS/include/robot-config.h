using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor LeftMotor;
extern motor RightMotor;
extern motor LeftMotor2;
extern motor RightMotor2;
extern motor_group Intake;
extern digital_out Indexer;
extern digital_out Expansion;
extern motor_group Flywheel;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );