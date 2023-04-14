#include "main.h"

///// ez::print_to_screen
// if (master.get_digital(DIGITAL_L1))
    //set_intake(127);
  //else if (master.get_digital(DIGITAL_L2))
   // set_intake(-127);
  //else
    //set_intake(0);
//}
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////
/// CONFIG
pros::Motor intake(21, MOTOR_GEARSET_06, true);
pros::Motor fw(11, MOTOR_GEARSET_06, true);
pros::ADIDigitalOut expansion('B');
pros::ADIDigitalOut blooper('A');
bool blooperToggle = false;
///LEDSTUFF
 void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

// Chassis constructor
Drive chassis (
  {4, -14, 15}
  ,{-9, 19, 20}
  ,18
  ,4
  ,600
  ,2.333333
);



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::print_ez_template();
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.
  ///LEDSTUFF
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	sylib::initialize();
	pros::lcd::register_btn1_cb(on_center_button);
  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("Example Drive\n\nDrive forward and come back.", drive_example),
    Auton("Example Turn\n\nTurn 3 times.", turn_example),
    Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
    Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
    Auton("Swing Example\n\nSwing, drive, swing.", swing_example),
    Auton("Combine all 3 movements", combining_movements),
    Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}



/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}



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
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
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



 /////CALLBACKS
 void tripleShot()
{
	pros::ADIDigitalOut indexer('C');
  indexer.set_value(true);
  pros::delay(80);
  indexer.set_value(false);
  pros::delay(100);
  indexer.set_value(true);
  pros::delay(80);
  indexer.set_value(false);
  pros::delay(150);
  indexer.set_value(true);
  pros::delay(80);
  indexer.set_value(false);
  pros::delay(100);
	
}
void singleShot()
{
	pros::ADIDigitalOut indexer('C');
	indexer.set_value(true);
	pros::delay(80);
	indexer.set_value(false);
	pros::delay(100);
}

void opcontrol() {
  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);
  ///LEDSTUFF
auto addrled1 = sylib::Addrled(22, 7, -8);
	auto addrled2 = sylib::Addrled(22, 8, -8);

	addrled1.set_all(0xFF2E02);
	addrled2.set_all(0xFF2E02);

	addrled1.pulse(0x00AF47, 2, 8);
	addrled2.pulse(0x00AF47, 2, 8);
	std::uint32_t clock = sylib::millis();

  ////

  while (true) { 
    ///LEDSTUFF
    pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
						 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
						 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

    chassis.tank();
/////FLYWHEEL
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      fw.move_voltage(11000); 
    }
  else
		{
			fw.brake();
		}
///TRIPLESHOT
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    tripleShot();
			pros::delay(500);
    }
//////SINGLESHOT
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
    singleShot();
			pros::delay(500);
       }
/////EXPANSION
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
    expansion.set_value(true); //expand
    pros::delay(500);
    expansion.set_value(false); //retract piston after shhoting
       }
///////INTAKE
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    intake.move_voltage(12000); }
  else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
    intake.move_velocity(-12000);
  } else {
    intake.brake();
  }
  ///////BLOOPER
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {              
     blooperToggle = !blooperToggle;
}

if (blooperToggle) {
   blooper.set_value(true);
} else {
   blooper.set_value(false);
}
}

    sylib::delay_until(&clock, 10);

    
    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
