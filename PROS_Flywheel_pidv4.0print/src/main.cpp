#include "main.h"

#define Flywheel_Motor_Port 6

pros::Motor Flywheel_Motor (Flywheel_Motor_Port,pros::E_MOTOR_GEAR_BLUE);
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
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

// Variables you change
float Flywheel_Motor_Derivative_Cutoff_Frequency; // a value ONLY between 0 and 1, other values will get clamped
float Flywheel_Motor_Cutoff_Frequency; // a value ONLY between 0 and 1, other values will get clamped
int Flywheel_Motor_Integral_Limit;
float Target_Flywheel_Motor_RPM;
float Flywheel_Motor_Kp;
float Flywheel_Motor_Ki;
float Flywheel_Motor_Kd;
float Flywheel_Motor_Kf;

// Don't touch these
bool Button_Pressed                                = false;
bool Task_Ended                                    = false;
static char File_Seperator                         = ',';
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
                 float f_FM_derCoff_Frequency, float f_FM_Kf)
{
                Flywheel_Motor_Integral_Limit         = i_FM_Integral_Limit;
                Target_Flywheel_Motor_RPM             = f_TFM_RPM/6;                
                Flywheel_Motor_Kp                     = f_FM_Kp;
                Flywheel_Motor_Ki                     = f_FM_Ki;
                Flywheel_Motor_Kd                     = f_FM_Kd;                
                Flywheel_Motor_Feedforwarded_Velocity = Target_Flywheel_Motor_RPM * f_FM_Kf;

                Flywheel_Motor_Cutoff_Frequency = std::clamp(f_FM_Coff_Frequency, static_cast<float>(0), static_cast<float>(1));
								Flywheel_Motor_Derivative_Cutoff_Frequency = std::clamp(f_FM_derCoff_Frequency, static_cast<float>(0), static_cast<float>(1));
}

inline void get_and_filter_motor_velocity (double d_Flywheel_Motor_Velocity)
{
  Flywheel_Motor_Filtered_Velocity = Flywheel_Motor_Cutoff_Frequency * d_Flywheel_Motor_Velocity 
                                     + (1 - Flywheel_Motor_Cutoff_Frequency) * Previous_Flywheel_Motor_Filtered_Velocity;
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
    Flywheel_Motor.move_voltage(Flywheel_MotorP6N_sentVoltage);
  }
  else if (Target_Flywheel_Motor_RPM == 0) 
  {
    Flywheel_Motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Flywheel_Motor.brake();
		while (1)
		{
			if (Flywheel_Motor.is_stopped())
			{
		  	Flywheel_Motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        // Task_Ended = true; (use case is to continually force the motor to be stopped so this is not needed)
				break;
			}
			pros::delay(20);
		}
  }
  else 
  {
    Flywheel_MotorP6N_sentVoltage = (Flywheel_Motor_Feedforwarded_Velocity + Flywheel_Motor_Kp * Flywheel_Motor_Error + Flywheel_Motor_Ki * Flywheel_Motor_Integral - Flywheel_Motor_Kd * Flywheel_Motor_Filtered_Derivative);
    Flywheel_Motor.move_voltage(Flywheel_MotorP6N_sentVoltage);
  }
}

inline void record_previous_values()
{
  Previous_Flywheel_Motor_Filtered_Derivative = Flywheel_Motor_Filtered_Derivative;
  Previous_Flywheel_Motor_Filtered_Velocity   = Flywheel_Motor_Filtered_Velocity;
  Previous_Flywheel_Motor_error               = Flywheel_Motor_Error;
}

void flywheel_pid()
{
  get_and_filter_motor_velocity(Flywheel_Motor.get_actual_velocity());
  set_motor_error();
  process_speed();
  set_motor_volt();
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
  std::ofstream file_handler("result_log.csv", std::ofstream::out); // creates a "result_log.csv" file
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

int main_fcn()
{
	master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X); // Allows for the ability for the flywheel code to be ended
  pros::delay(1000);
  // Target_Flywheel_Motor_RPM is between 0 and 3600
	set_values(Target_Flywheel_Motor_RPM, Flywheel_Motor_Kp, Flywheel_Motor_Ki, Flywheel_Motor_Kd, Flywheel_Motor_Integral_Limit, 
             Flywheel_Motor_Cutoff_Frequency, Flywheel_Motor_Derivative_Cutoff_Frequency, Flywheel_Motor_Kf);
  File_text << "Script counter,Delta time/seconds,Time elapsed/seconds,"
               "Flywheel motor RPM,Target Flywheel motor RPM,Flywheel motor filtered velocity,"
               "Flywheel motor proportional gain,Flywheel motor integral gain,Flywheel motor derivative gain,Flywheel motor feedforwarded gain,"
               "Flywheel motor commanded voltage/volts,Flywheel motor integral limit,"
               "Flywheel motor current/amperes,Flywheel motor voltage/volts,Flywheel motor power/watts,Flywheel motor torque/newton meters,"
               "Flywheel motor efficiency/pct,Flywheel motor temperature/celsius,"
               "Flywheel motor Kp,Flywheel motor Ki,Flywheel motor Kd,Flywheel motor Kf,"
               "Flywheel motor cutoff frequency,Flywheel motor derivative cutoff frequency"; // writes the collumn titles of the csv file
  qLog.push(File_text.str());
  pros::delay(500);
	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
	{
		std::cout << "The programme has been cancelled." << std::endl;
    pros::lcd::clear();  
		pros::lcd::print(0, "The programme has been cancelled.");
    Task_Ended = true;
	  return 1;
	}
	else 
	{
		// t_Timer.placeHardMark();
	  // t_Timer.getDtFromHardMark().convert(okapi::millisecond);
  	start_timer_time = pros::c::micros();
		start_time = pros::c::micros();
		get_and_filter_motor_velocity(Flywheel_Motor.get_actual_velocity());
		set_motor_error();
		set_motor_volt();
		record_previous_values();
		Script_Counter = 1;
		out_stream =  std::to_string(Script_Counter) + File_Seperator;
		delta_time = compute_delta_time(); 					
		File_text << out_stream << delta_time << File_Seperator << delta_time << File_Seperator
							<< (Flywheel_Motor.get_actual_velocity()*6) << File_Seperator 
							<< Target_Flywheel_Motor_RPM << File_Seperator << (Flywheel_Motor_Filtered_Velocity*6) << File_Seperator
							<< (Flywheel_Motor_Error*Flywheel_Motor_Kp) << File_Seperator << (Flywheel_Motor_Integral*Flywheel_Motor_Ki) << File_Seperator
							<< (-Flywheel_Motor_Filtered_Derivative*Flywheel_Motor_Kd) << File_Seperator << Flywheel_Motor_Feedforwarded_Velocity << File_Seperator
							<< Flywheel_MotorP6N_sentVoltage << File_Seperator << Flywheel_Motor_Integral_Limit << File_Seperator 
							<< Flywheel_Motor.get_current_draw() << File_Seperator << Flywheel_Motor.get_voltage() << File_Seperator
							<< Flywheel_Motor.get_power() << File_Seperator << Flywheel_Motor.get_torque() << File_Seperator
							<< Flywheel_Motor.get_efficiency() << File_Seperator << Flywheel_Motor.get_temperature() << File_Seperator  
							<< Flywheel_Motor_Kp << File_Seperator << Flywheel_Motor_Ki << File_Seperator << Flywheel_Motor_Kd << Flywheel_Motor_Kf << File_Seperator
							<< Flywheel_Motor_Cutoff_Frequency << File_Seperator << Flywheel_Motor_Derivative_Cutoff_Frequency << File_Seperator; 					
		qLog.push(File_text.str());
		delta_time = compute_delta_time();		
	}

	while (!master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
  {
    delta_time = compute_delta_time();
    start_time = compute_start_time();
    flywheel_pid();
    File_text << ++Script_Counter << File_Seperator << compute_delta_time() << File_Seperator << compute_start_time() << File_Seperator
              << (Flywheel_Motor.get_actual_velocity()*6) << File_Seperator 
              << Target_Flywheel_Motor_RPM << File_Seperator << (Flywheel_Motor_Filtered_Velocity*6) << File_Seperator
         	    << (Flywheel_Motor_Error*Flywheel_Motor_Kp) << File_Seperator << (Flywheel_Motor_Integral*Flywheel_Motor_Ki) << File_Seperator
          	  << (-Flywheel_Motor_Filtered_Derivative*Flywheel_Motor_Kd) << File_Seperator << Flywheel_Motor_Feedforwarded_Velocity << File_Seperator
  	          << Flywheel_MotorP6N_sentVoltage << File_Seperator << Flywheel_Motor_Integral_Limit << File_Seperator 
    	        << Flywheel_Motor.get_current_draw() << File_Seperator << Flywheel_Motor.get_voltage() << File_Seperator
      	      << Flywheel_Motor.get_power() << File_Seperator << Flywheel_Motor.get_torque() << File_Seperator
        	    << Flywheel_Motor.get_efficiency() << File_Seperator << Flywheel_Motor.get_temperature() << File_Seperator  
          	  << Flywheel_Motor_Kp << File_Seperator << Flywheel_Motor_Ki << File_Seperator << Flywheel_Motor_Kd << Flywheel_Motor_Kf << File_Seperator
            	<< Flywheel_Motor_Cutoff_Frequency << File_Seperator << Flywheel_Motor_Derivative_Cutoff_Frequency << File_Seperator;  
    qLog.push(File_text.str());  
  }					
	delta_time = compute_delta_time();			 
	if (Button_Pressed)
  {
    Flywheel_Motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    Flywheel_Motor.brake();
    write_file_from_queue(qLog);
  }
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
  pros::Task flywheel_test (main_fcn, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "main_t");
  pros::Task::delay_until(&now,1000);
  while (!Task_Ended)
  {
    now = pros::millis();
    pros::Task::delay_until(&now,1000);
  }
  pros::lcd::print(5, "The programme has finished excecuting.");
  return;
}