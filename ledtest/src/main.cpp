#include "main.h"

pros::ADIDigitalOut expansion('A');
pros::ADIDigitalOut blooper('B');
bool blooperToggle = false;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	sylib::initialize();
	pros::lcd::register_btn1_cb(on_center_button);
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
void autonomous() {}

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
void opcontrol()
{
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor driveLeftBack(15);
	pros::Motor driveLeftFrontBottom(14);
	pros::Motor driveLeftFrontTop(4, true);
	pros::Motor driveRightBack(20);
	pros::Motor driveRightFrontBottom(19);
	pros::Motor driveRightFrontTop(9, true);
	pros::Motor intake(10, true);
	pros::Motor fw(6, true);
	pros::ADIDigitalOut shotgun('B');

	auto addrled1 = sylib::Addrled(22, 7, -8);
	auto addrled2 = sylib::Addrled(22, 8, -8);

	addrled1.set_all(0xFF2E02);
	addrled2.set_all(0xFF2E02);

	addrled1.pulse(0x00AF47, 2, 8);
	addrled2.pulse(0x00AF47, 2, 8);

	shotgun.set_value(false);

	std::uint32_t clock = sylib::millis();

	while (true)
	{
		clock = sylib::millis();
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
						 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
						 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		int left = master.get_analog(ANALOG_LEFT_Y) * -1;
		int right = master.get_analog(ANALOG_RIGHT_Y);

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			fw.move_voltage(11000);
		}
		else
		{
			fw.brake();
		}
		/// TRIPLESHOT
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			tripleShot();
			pros::delay(500);
		}
		//////SINGLESHOT
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			singleShot();
			pros::delay(100);
		}
		/////EXPANSION
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
		{
			expansion.set_value(true); // expand
			pros::delay(500);
			expansion.set_value(false); // retract piston after shhoting
		}
		///////INTAKE
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			intake.move_voltage(12000);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
		{
			intake.move_velocity(-12000);
		}
		else
		{
			intake.brake();
		}
		///////BLOOPER
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
		{
			blooperToggle = !blooperToggle;
		}

		if (blooperToggle)
		{
			blooper.set_value(true);
		}
		else
		{
			blooper.set_value(false);
		}

		driveLeftBack = left;
		driveLeftFrontBottom = left;
		driveLeftFrontTop = left;
		driveRightBack = right;
		driveRightFrontBottom = right;
		driveRightFrontTop = right;
	}

	sylib::delay_until(&clock, 20);

	pros::delay(20);
}
