#include "main.h"

#define Flywheel_Motor_Port 2
#define File_Seperator ","

sylib::Motor Flywheel_Motor (Flywheel_Motor_Port,3600, false);
pros::Controller master (CONTROLLER_MASTER);

// okapi::Timer t_Timer; (use if needed)

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

// Template to find the sign of a variable
template <typename T> int sgn(T val)
{
	return (T(0) < val) - (val < T(0));
}

// Variables you change
float Flywheel_Motor_Derivative_Cutoff_Frequency; // a value ONLY between 0 and 1, other values will get clamped
float Flywheel_Motor_Cutoff_Frequency; // a value ONLY between 0 and 1, other values will get clamped
int Flywheel_Motor_Integral_Limit;
float Target_Flywheel_Motor_RPM;
float Flywheel_Motor_Kp;
float Flywheel_Motor_Ki;
float Flywheel_Motor_Kd;
float Flywheel_Motor_Ks;
float Flywheel_Motor_Kv;

// Don't touch these
bool Task_Ended                                    = false;
int Script_Counter                                 = 1;
double Flywheel_Motor_Filtered_Velocity            = 0;
double Flywheel_Motor_Error                        = 0;
double Flywheel_Motor_Integral                     = 0;
double Flywheel_Motor_Filtered_Derivative          = 0;
double Flywheel_Motor_Feedforwarded_Velocity       = 0;
double Previous_Flywheel_Motor_Filtered_Derivative = 0;
double Previous_Flywheel_Motor_Filtered_Velocity   = 0;
double Previous_Flywheel_Motor_error               = 0;
double Flywheel_MotorP6N_sentVoltage               = 0;
double start_timer_time                            = 0;
double start_time                                  = 0;
double delta_time                                  = 0;
std::queue <std::string> qLog;
std::ostringstream File_text;
std::string out_stream;

void set_values (float f_TFM_RPM, float f_FM_Kp, float f_FM_Ki, float f_FM_Kd,
								 int i_FM_Integral_Limit, float f_FM_Coff_Frequency,
								 float f_FM_derCoff_Frequency, float f_FM_Kv, float f_FM_Ks)
{
								Flywheel_Motor_Integral_Limit         = i_FM_Integral_Limit;
								Target_Flywheel_Motor_RPM             = f_TFM_RPM;
								Flywheel_Motor_Kp                     = f_FM_Kp;
								Flywheel_Motor_Ki                     = f_FM_Ki;
								Flywheel_Motor_Kd                     = f_FM_Kd;
								Flywheel_Motor_Ks                     = sgn(Target_Flywheel_Motor_RPM) * f_FM_Ks;
								Flywheel_Motor_Feedforwarded_Velocity = Target_Flywheel_Motor_RPM * f_FM_Kv + Flywheel_Motor_Ks;

								Flywheel_Motor_Cutoff_Frequency = std::clamp(f_FM_Coff_Frequency, static_cast<float>(0), static_cast<float>(1));
								Flywheel_Motor_Derivative_Cutoff_Frequency = std::clamp(f_FM_derCoff_Frequency, static_cast<float>(0), static_cast<float>(1));
}

inline void set_only_TFM_RPM(float f_TFM_RPM)
{
	Target_Flywheel_Motor_RPM = f_TFM_RPM;
}

inline void filter_motor_velocity (double d_Flywheel_Motor_Velocity)
{
	Flywheel_Motor_Filtered_Velocity = Flywheel_Motor_Cutoff_Frequency * d_Flywheel_Motor_Velocity
																		 + (1 - Flywheel_Motor_Cutoff_Frequency) * Previous_Flywheel_Motor_Filtered_Velocity;
}

inline double return_and_filter_motor_velocity (double d_Flywheel_Motor_Velocity)
{
	 return (Flywheel_Motor_Cutoff_Frequency * d_Flywheel_Motor_Velocity +
					(1 - Flywheel_Motor_Cutoff_Frequency) * Previous_Flywheel_Motor_Filtered_Velocity);
}

inline void set_motor_error()
{
	Flywheel_Motor_Error = Target_Flywheel_Motor_RPM - Flywheel_Motor_Filtered_Velocity;
}

void process_speed()
{
	Flywheel_Motor_Integral = ((Flywheel_Motor_Error == 0) || (fabs(Flywheel_Motor_Error) > Flywheel_Motor_Integral_Limit) ||
														((Target_Flywheel_Motor_RPM > 0) && (Previous_Flywheel_Motor_error > 0) && (Flywheel_Motor_Error < 0)) ||
														((Target_Flywheel_Motor_RPM < 0) && (Previous_Flywheel_Motor_error < 0) && (Flywheel_Motor_Error > 0))) ? 0
														: Flywheel_Motor_Integral + Flywheel_Motor_Error * delta_time;

	Flywheel_Motor_Filtered_Derivative = Flywheel_Motor_Derivative_Cutoff_Frequency
																			 * ((Flywheel_Motor_Filtered_Velocity - Previous_Flywheel_Motor_Filtered_Velocity)/delta_time)
																			 + (1 - Flywheel_Motor_Derivative_Cutoff_Frequency) * Previous_Flywheel_Motor_Filtered_Derivative;
}

void set_motor_volt()
{
	if (Target_Flywheel_Motor_RPM > 0)
	{
		Flywheel_MotorP6N_sentVoltage = (Flywheel_Motor_Feedforwarded_Velocity + Flywheel_Motor_Kp * Flywheel_Motor_Error + Flywheel_Motor_Ki * Flywheel_Motor_Integral - Flywheel_Motor_Kd * Flywheel_Motor_Filtered_Derivative);
		Flywheel_Motor.set_voltage(Flywheel_MotorP6N_sentVoltage);
	}
	else if (Target_Flywheel_Motor_RPM == 0)
	{
		Flywheel_Motor.set_braking_mode(kV5MotorBrakeModeHold);
		Flywheel_Motor.stop();
		while (1)
		{
			if (Flywheel_Motor.is_stopped())
			{
				Flywheel_Motor.set_braking_mode(kV5MotorBrakeModeCoast);
				// Task_Ended = true; (use case is to continually force the motor to be stopped so this is not needed)
				break;
			}
			pros::delay(20);
		}
	}
	else
	{
		Flywheel_MotorP6N_sentVoltage = (Flywheel_Motor_Feedforwarded_Velocity + Flywheel_Motor_Kp * Flywheel_Motor_Error + Flywheel_Motor_Ki * Flywheel_Motor_Integral - Flywheel_Motor_Kd * Flywheel_Motor_Filtered_Derivative);
		Flywheel_Motor.set_voltage(Flywheel_MotorP6N_sentVoltage);
	}
}

inline void record_previous_values()
{
	Previous_Flywheel_Motor_Filtered_Derivative = Flywheel_Motor_Filtered_Derivative;
	Previous_Flywheel_Motor_Filtered_Velocity   = Flywheel_Motor_Filtered_Velocity;
	Previous_Flywheel_Motor_error               = Flywheel_Motor_Error;
}

inline void set_motor_volt_when_TFM_RPM_is_positive()
{
	Flywheel_MotorP6N_sentVoltage = (Flywheel_Motor_Feedforwarded_Velocity + Flywheel_Motor_Kp * Flywheel_Motor_Error + Flywheel_Motor_Ki * Flywheel_Motor_Integral - Flywheel_Motor_Kd * Flywheel_Motor_Filtered_Derivative);
	Flywheel_Motor.set_voltage(Flywheel_MotorP6N_sentVoltage);
}

inline void set_motor_volt_when_TFM_RPM_is_negative()
{
	Flywheel_MotorP6N_sentVoltage = (Flywheel_Motor_Feedforwarded_Velocity + Flywheel_Motor_Kp * Flywheel_Motor_Error + Flywheel_Motor_Ki * Flywheel_Motor_Integral - Flywheel_Motor_Kd * Flywheel_Motor_Filtered_Derivative);
	Flywheel_Motor.set_voltage(Flywheel_MotorP6N_sentVoltage);
}

inline void set_motor_volt_when_TFM_RPM_is_0()
{
	Flywheel_Motor.set_braking_mode(kV5MotorBrakeModeHold);
	Flywheel_Motor.stop();
	while (1)
	{
		if (Flywheel_Motor.is_stopped())
		{
			Flywheel_Motor.set_braking_mode(kV5MotorBrakeModeCoast);
			// Task_Ended = true; (use case is to continually force the motor to be stopped so this is not needed)
			break;
		}
		pros::delay(20);
	}
}

void flywheel_pid_when_TFM_is_positive()
{
	filter_motor_velocity(Flywheel_Motor.get_velocity());
	set_motor_error();
	process_speed();
	set_motor_volt_when_TFM_RPM_is_positive();
	record_previous_values();
}

void flywheel_pid_when_TFM_is_0()
{
	filter_motor_velocity(Flywheel_Motor.get_velocity());
	set_motor_error();
	process_speed();
	set_motor_volt_when_TFM_RPM_is_0();
	record_previous_values();
}

void flywheel_pid_when_TFM_is_negative()
{
	filter_motor_velocity(Flywheel_Motor.get_velocity());
	set_motor_error();
	process_speed();
	set_motor_volt_when_TFM_RPM_is_negative();
	record_previous_values();
}

void write_file_from_queue(std::queue<std::string> q)
{
	if (!pros::usd::is_installed()) // checks if an sdcard is installed and displays an error message
	{
		std::cout << "Please insert the SD card. Try again." << std::endl;
		pros::lcd::clear();
		pros::lcd::print(0, "Please insert the sdcard.");
		pros::lcd::print(1, "Waiting until the SD card is installed...");
		std::cout << "Waiting until the SD card is installed..." << std::endl;
		if (!pros::usd::is_installed())
		{
			std::uint32_t now = pros::millis();
			pros::Task::delay_until(&now,100);
			if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
			{
				std::cout << "The Printing has been cancelled." << std::endl;
				pros::lcd::clear();
				pros::lcd::print(0, "The printing has been cancelled.");
				return;
			}
			while (!pros::usd::is_installed())
			{
				now = pros::millis();
				pros::Task::delay_until(&now,100);
				if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
				{
					std::cout << "The Printing has been cancelled." << std::endl;
					pros::lcd::clear();
					pros::lcd::print(0, "The printing has been cancelled.");
					return;
				}
			}
		}
		pros::lcd::clear();
	}
	std::cout << "The Printing has commenced." << std::endl;
	pros::lcd::clear();
	pros::lcd::print(0, "The printing has commenced.");
	std::ofstream file_handler("/usd/result_log.csv", std::ofstream::out); // creates a "result_log.csv" file
	while (!q.empty()) // repeatedly checks if a queue is empty and writes the queue's contents to a sdcard file
	{
		file_handler << q.front() << std::endl;
		q.pop();
	}
	file_handler.close();
	return;
}

inline double compute_delta_time()
{
	return (pros::c::micros() - start_time);
}

inline double compute_start_time()
{
	return (pros::c::micros() - start_timer_time);
}

inline void create_initial_file_text_string_stream()
{
	Script_Counter = 1;
	out_stream =  std::to_string(Script_Counter) + File_Seperator;
	delta_time = compute_delta_time();
	File_text << out_stream << delta_time << File_Seperator << delta_time << File_Seperator
						<< Flywheel_Motor.get_velocity() << File_Seperator
						<< Target_Flywheel_Motor_RPM << File_Seperator << return_and_filter_motor_velocity(Flywheel_Motor.get_velocity()) << File_Seperator
						<< (Flywheel_Motor_Error*Flywheel_Motor_Kp) << File_Seperator << (Flywheel_Motor_Integral*Flywheel_Motor_Ki) << File_Seperator
						<< (-Flywheel_Motor_Filtered_Derivative*Flywheel_Motor_Kd) << File_Seperator << Flywheel_Motor_Feedforwarded_Velocity << File_Seperator
						<< Flywheel_MotorP6N_sentVoltage << File_Seperator << Flywheel_Motor_Integral_Limit << File_Seperator
						<< Flywheel_Motor.get_amps() << File_Seperator << Flywheel_Motor.get_watts()/Flywheel_Motor.get_amps() << File_Seperator
						<< Flywheel_Motor.get_watts() << File_Seperator << Flywheel_Motor.get_torque() << File_Seperator
						<< Flywheel_Motor.get_efficiency() << File_Seperator << Flywheel_Motor.get_temperature() << File_Seperator
						<< Flywheel_Motor_Kp << File_Seperator << Flywheel_Motor_Ki << File_Seperator << Flywheel_Motor_Kd << File_Seperator
						<< Flywheel_Motor_Ks << File_Seperator << Flywheel_Motor_Kv << File_Seperator
						<< Flywheel_Motor_Cutoff_Frequency << File_Seperator << Flywheel_Motor_Derivative_Cutoff_Frequency << File_Seperator;
}

inline void create_looped_file_text_string_stream()
{
	File_text << ++Script_Counter << File_Seperator << compute_delta_time() << File_Seperator << compute_start_time() << File_Seperator
						<< Flywheel_Motor.get_velocity() << File_Seperator
						<< Target_Flywheel_Motor_RPM << File_Seperator << return_and_filter_motor_velocity(Flywheel_Motor.get_velocity()) << File_Seperator
						<< (Flywheel_Motor_Error*Flywheel_Motor_Kp) << File_Seperator << (Flywheel_Motor_Integral*Flywheel_Motor_Ki) << File_Seperator
						<< (-Flywheel_Motor_Filtered_Derivative*Flywheel_Motor_Kd) << File_Seperator << Flywheel_Motor_Feedforwarded_Velocity << File_Seperator
						<< Flywheel_MotorP6N_sentVoltage << File_Seperator << Flywheel_Motor_Integral_Limit << File_Seperator
						<< Flywheel_Motor.get_amps() << File_Seperator << Flywheel_Motor.get_watts()/Flywheel_Motor.get_amps() << File_Seperator
						<< Flywheel_Motor.get_watts() << File_Seperator << Flywheel_Motor.get_torque() << File_Seperator
						<< Flywheel_Motor.get_efficiency() << File_Seperator << Flywheel_Motor.get_temperature() << File_Seperator
						<< Flywheel_Motor_Kp << File_Seperator << Flywheel_Motor_Ki << File_Seperator << Flywheel_Motor_Kd << File_Seperator
						<< Flywheel_Motor_Ks << File_Seperator << Flywheel_Motor_Kv << File_Seperator
						<< Flywheel_Motor_Cutoff_Frequency << File_Seperator << Flywheel_Motor_Derivative_Cutoff_Frequency << File_Seperator;
}

void main_fcn()
{
	if (1)
	{
		bool Start_Up_Get_Digital = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X); // Allows for the ability for the flywheel code to be ended
	} // Purpose of if is to lower the scope of Start_Up_Get_Digital
	pros::delay(1500);
	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
	{
		std::cout << "The programme has been cancelled." << std::endl;
		pros::lcd::clear();
		pros::lcd::print(0, "The programme has been cancelled.");
		Task_Ended = true;
		return;
	}
	else
	{
		File_text << "Script counter,Delta time/seconds,Time elapsed/seconds,"
								 "Flywheel motor RPM,Target Flywheel motor RPM,Flywheel motor filtered velocity,"
								 "Flywheel motor proportional gain,Flywheel motor integral gain,Flywheel motor derivative gain,Flywheel motor feedforwarded gain,"
								 "Flywheel motor commanded voltage/volts,Flywheel motor integral limit,"
								 "Flywheel motor current/amperes,Flywheel motor voltage/volts,Flywheel motor power/watts,Flywheel motor torque/newton meters,"
								 "Flywheel motor efficiency/pct,Flywheel motor temperature/celsius,"
								 "Flywheel motor Kp,Flywheel motor Ki,Flywheel motor Kd,Flywheel motor Ks, Flywheel motor Kv"
								 "Flywheel motor cutoff frequency,Flywheel motor derivative cutoff frequency"; // writes the collumn titles of the csv file
		qLog.push(File_text.str());
		// Target_Flywheel_Motor_RPM is between 0 and 3600
		set_values(2835, 10, 10, 0, 500000000,
							 1, 1, 10,10); // some random values
		if (Target_Flywheel_Motor_RPM < 0)
		{
			// t_Timer.placeHardMark();
			// t_Timer.getDtFromHardMark().convert(okapi::millisecond);
			start_timer_time = pros::c::micros();
			start_time = pros::c::micros();
			filter_motor_velocity(Flywheel_Motor.get_velocity());
			set_motor_error();
			set_motor_volt_when_TFM_RPM_is_positive();
			record_previous_values();
			Script_Counter = 1;
			out_stream =  std::to_string(Script_Counter) + File_Seperator;
			delta_time = compute_delta_time();
			create_initial_file_text_string_stream();
			qLog.push(File_text.str());
			delta_time = compute_delta_time();
			while (!master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
			{
				delta_time = compute_delta_time();
				start_time = compute_start_time();
				flywheel_pid_when_TFM_is_positive();
				create_looped_file_text_string_stream();
				qLog.push(File_text.str());
			}
		}
		else if (Target_Flywheel_Motor_RPM == 0)
		{
			// t_Timer.placeHardMark();
			// t_Timer.getDtFromHardMark().convert(okapi::millisecond);
			start_timer_time = pros::c::micros();
			start_time = pros::c::micros();
			filter_motor_velocity(Flywheel_Motor.get_velocity());
			set_motor_error();
			set_motor_volt_when_TFM_RPM_is_0();
			record_previous_values();
			Script_Counter = 1;
			out_stream =  std::to_string(Script_Counter) + File_Seperator;
			delta_time = compute_delta_time();
			create_initial_file_text_string_stream();
			qLog.push(File_text.str());
			delta_time = compute_delta_time();
			while (!master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
			{
				delta_time = compute_delta_time();
				start_time = compute_start_time();
				flywheel_pid_when_TFM_is_0();
				create_looped_file_text_string_stream();
				qLog.push(File_text.str());
			}
		}
		else
		{
			// t_Timer.placeHardMark();
			// t_Timer.getDtFromHardMark().convert(okapi::millisecond);
			start_timer_time = pros::c::micros();
			start_time = pros::c::micros();
			filter_motor_velocity(Flywheel_Motor.get_velocity());
			set_motor_error();
			set_motor_volt_when_TFM_RPM_is_negative();
			record_previous_values();
			Script_Counter = 1;
			out_stream =  std::to_string(Script_Counter) + File_Seperator;
			delta_time = compute_delta_time();
			create_initial_file_text_string_stream();
			qLog.push(File_text.str());
			delta_time = compute_delta_time();
			while (!master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
			{
				delta_time = compute_delta_time();
				start_time = compute_start_time();
				flywheel_pid_when_TFM_is_negative();
				create_looped_file_text_string_stream();
				qLog.push(File_text.str());
			}
		}
	}
	delta_time = compute_delta_time();
	Flywheel_Motor.set_braking_mode(kV5MotorBrakeModeCoast);
	Flywheel_Motor.stop();
	write_file_from_queue(qLog);
	Task_Ended = true;
	return;
}

int continuation_of_flywheel_pid()
{
	if (Target_Flywheel_Motor_RPM<0)
	{
		while (!master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{
			delta_time = compute_delta_time();
			start_time = compute_start_time();
			flywheel_pid_when_TFM_is_positive();
			create_looped_file_text_string_stream();
			qLog.push(File_text.str());
		}
	}
	else if (Target_Flywheel_Motor_RPM == 0)
	{
		while (!master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{
			delta_time = compute_delta_time();
			start_time = compute_start_time();
			flywheel_pid_when_TFM_is_0();
			create_looped_file_text_string_stream();
			qLog.push(File_text.str());
		}
	}
	else
	{
		while (!master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{
			delta_time = compute_delta_time();
			start_time = compute_start_time();
			flywheel_pid_when_TFM_is_negative();
			create_looped_file_text_string_stream();
			qLog.push(File_text.str());
		}
	}
	delta_time = compute_delta_time();
	Flywheel_Motor.set_braking_mode(kV5MotorBrakeModeCoast);
	Flywheel_Motor.stop();
	write_file_from_queue(qLog);
	Task_Ended = true;
	return 0;
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::delay(500); // Stop the user from doing anything while legacy ports configure.
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);
	sylib::initialize();
	pros::delay(200); // little break just in case something has not finished
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
void opcontrol()
{
	std::uint32_t now = pros::millis();
	pros::Task flywheel_test(main_fcn, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "main_t");
	pros::Task::delay_until(&now,20000);
	flywheel_test.suspend();
	flywheel_test.remove();
	set_values(2835, 10, 10, 0, 5000,
						 1, 1, 10, 10); // some new values
	// or do only this
	set_only_TFM_RPM(2835);
	// and then start the continuation task
	pros::Task flywheel_continue (continuation_of_flywheel_pid, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "main_new");
	// repeat this process for changing RPM agai
	while (!Task_Ended)
	{
		now = pros::millis();
		pros::Task::delay_until(&now,1000);
	}
	pros::lcd::print(5, "The programme has finished excecuting.");
	return;
}