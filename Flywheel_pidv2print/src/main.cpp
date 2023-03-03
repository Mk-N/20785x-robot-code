/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\natco                                            */
/*    Created:      Thu Feb 09 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FM1_8                motor         8               
// FM2_9                motor         4               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <queue>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace vex;
using namespace std;

// Variables you change
float Target_Flywheel1_RPM;
float Target_Flywheel2_RPM;
float Motor1_Kp;
float Motor1_Ki;
float Motor1_Kd;
float Motor2_Kp; 
float Motor2_Ki;
float Motor2_Kd;
int Motor1_Integral_Limit;
int Motor2_Integral_Limit;

// Don't touch these
double Motor1_Error = 0;
double Motor2_Error = 0;
double Motor1_Integral = 0;
double Motor2_Integral = 0;
double Motor1_Derivative = 0;
double Motor2_Derivative = 0;
double Previous_Motor1_Error = 0;
double Previous_Motor2_Error = 0;
double FM1_8_sentVoltage = 0;
double FM2_9_sentVoltage = 0;
double start_time = 0;
double delta_time = 0;
static bool Button_Pressed = false;
queue <string> qLog;
int script_counter = 1;
ostringstream File_text;
static char file_seperator = ',';

void set_values(float f_T1F_RPM, float f_T2R_RPM,
                float f_FM1_Kp, float f_FM1_Ki, float f_FM1_Kd,
                float f_FM2_Kp, float f_FM2_Ki, float f_FM2_Kd,
                int i_FM1_Integral_Limit, int i_FM2_Integral_Limit)
{
                Target_Flywheel1_RPM = f_T1F_RPM/6;
                Target_Flywheel2_RPM = f_T2R_RPM/6;
                Motor1_Kp = f_FM1_Kp;
                Motor1_Ki = f_FM1_Ki;
                Motor1_Kd = f_FM1_Kd;
                Motor2_Kp = f_FM2_Kp; 
                Motor2_Ki = f_FM2_Ki;
                Motor2_Kd = f_FM2_Kd;
                Motor1_Integral_Limit = i_FM1_Integral_Limit;
                Motor2_Integral_Limit = i_FM2_Integral_Limit;
}

void set_motor_error(double f_left_velocity, double f_right_velocity)
{
  Motor1_Error = Target_Flywheel1_RPM - f_left_velocity;
  Motor2_Error = Target_Flywheel2_RPM - f_right_velocity;
}

void process_speed()
{
  set_motor_error(FM1_8.velocity(rpm), FM2_9.velocity(rpm));
  Motor1_Integral = Motor1_Integral + Motor1_Error*delta_time;
  Motor2_Integral = Motor2_Integral + Motor2_Error*delta_time;
  Motor1_Derivative = (Motor1_Error - Previous_Motor1_Error)/delta_time;
  Motor2_Derivative = (Motor2_Error - Previous_Motor2_Error)/delta_time;    
  if ((Motor1_Error = 0) || (fabs(Motor1_Error) > Motor1_Integral_Limit))
  {
    Motor1_Integral = 0;
  }   
  if ((Motor2_Error = 0) || (fabs(Motor2_Error) > Motor2_Integral_Limit))
  {
    Motor2_Integral = 0;
  } 
}

void set_motor_volt()
{
  if (Target_Flywheel1_RPM > 0)
  {
    FM1_8_sentVoltage = (Motor1_Kp * Motor1_Error + Motor1_Ki * Motor1_Integral + Motor1_Kd * Motor1_Derivative);
    FM1_8.spin(fwd, FM1_8_sentVoltage, volt);
  }
  else if (Target_Flywheel1_RPM == 0) 
  {
    FM1_8.setStopping(hold);
    FM1_8.stop();
    FM1_8.setStopping(coast);
  }
  else 
  {
    FM1_8_sentVoltage = (Motor1_Kp * Motor1_Error + Motor1_Ki * Motor1_Integral + Motor1_Kd * Motor1_Derivative);
    FM1_8.spin(reverse, FM1_8_sentVoltage, volt);
  }
  if (Target_Flywheel2_RPM > 0)
  {
    FM2_9_sentVoltage  = (Motor2_Kp * Motor2_Error + Motor2_Ki * Motor2_Integral + Motor2_Kd * Motor2_Derivative);
    FM2_9.spin(fwd, FM2_9_sentVoltage, volt);
  }
  else if (Target_Flywheel2_RPM == 0) 
  {
    FM2_9.setStopping(hold);
    FM2_9.stop(); 
    FM2_9.setStopping(coast);
  }
  else 
  {
    FM2_9_sentVoltage  = (Motor2_Kp * Motor2_Error + Motor2_Ki * Motor2_Integral + Motor2_Kd * Motor2_Derivative);    
    FM2_9.spin(reverse, FM2_9_sentVoltage, volt);
  }
}

void calculate_previous_error()
{
  Previous_Motor1_Error = Motor1_Error;
  Previous_Motor2_Error = Motor2_Error;
} 

void flywheel_pid()
{
  process_speed();
  set_motor_volt();
  calculate_previous_error();
}

void set_button_pressed_true()
{
  Button_Pressed = true;
}

void write_file_from_queue(queue<string> q)
{
  if(!Brain.SDcard.isInserted())
  {
    cout << "Please insert the sdcard. Try again." << endl;
    Brain.Screen.print("Please insert the sdcard. Try again.");
    return;
  }
  ofstream file_handler("result_log.csv", ofstream::out);
  while(!q.empty())
  {
    file_handler << q.front() << endl;
    q.pop();
  } 
  file_handler.close();
}

int main() 
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  wait(1,sec);
  Controller1.ButtonX.pressed(set_button_pressed_true);
  // Target_Flywheel1_RPM and Target_Flywheel_RPM is between 0 and 3600
  // Keep them equal unless you are doing a double flywheel or require fine control
  set_values(Target_Flywheel1_RPM, Target_Flywheel2_RPM,
             Motor1_Kp, Motor1_Ki, Motor1_Kd,
             Motor2_Kp, Motor2_Ki, Motor2_Kd, 
             Motor1_Integral_Limit, Motor2_Integral_Limit);
  File_text << "Script counter,Delta time/seconds,Time elapsed/seconds,"
                "Motor1_RPM,Motor2_RPM,Target_Flywheel1_RPM,Target_Flywheel2_RPM,"
                "Motor1 proportional gain,Motor1 integral gain,Motor1 derivative gain,Motor1 commanded volt,Motor1 integral limit,"
                "Motor2 proportional gain,Motor2 integral gain,Motor2 derivative gain,Motor2 commanded volt,Motor2 integral limit,"
                "Motor1 current/amperes,Motor1 voltage/volts,Motor1 power/watts,Motor1 torque/Newton meters,"
                "Motor2 current/amperes,Motor2 voltage/volts,Motor2 power/watts,Motor2 torque/Newton meters,"
                "Motor1 effiency/pct, Motor1 temperature/celsius,Motor2 effiency/pct, Motor2 temperature/celsius";
  qLog.push(File_text.str());
  wait(500,msec);
  Brain.resetTimer();
  set_motor_error(FM1_8.velocity(rpm), FM2_9.velocity(rpm));
  set_motor_volt();
  calculate_previous_error();
  script_counter = 1;
  File_text << script_counter << file_seperator << Brain.timer(sec) << file_seperator << Brain.timer(sec) << file_seperator
            << (FM1_8.velocity(rpm)*6) << file_seperator << (FM2_9.velocity(rpm)*6) << file_seperator
            << Target_Flywheel1_RPM << file_seperator << Target_Flywheel2_RPM << file_seperator
            << (Motor1_Error*Motor1_Kp) << file_seperator << (Motor1_Integral*Motor1_Ki) << file_seperator
            << (Motor1_Derivative*Motor1_Kd) << file_seperator << FM1_8_sentVoltage << file_seperator
            << Motor1_Integral_Limit << file_seperator 
            << (Motor2_Error*Motor2_Kp) << file_seperator << (Motor2_Integral*Motor2_Ki) << file_seperator 
            << (Motor2_Derivative*Motor2_Kd) << file_seperator << FM2_9_sentVoltage << file_seperator 
            << Motor2_Integral_Limit << file_seperator
            << FM1_8.current() << file_seperator << FM1_8.voltage() << file_seperator
            << FM1_8.power() << file_seperator << FM1_8.torque() << file_seperator
            << FM2_9.current() << file_seperator << FM2_9.voltage() << file_seperator
            << FM2_9.power() << file_seperator << FM2_9.torque() << file_seperator
            << FM1_8.efficiency() << file_seperator << FM1_8.temperature(celsius) << file_seperator  
            << FM2_9.efficiency() << file_seperator << FM2_9.temperature(celsius);  
  qLog.push(File_text.str());
  delta_time = Brain.timer(sec);

  while(!Button_Pressed)
  {
    start_time = Brain.timer(sec);
    flywheel_pid();
    script_counter++;
    File_text << script_counter << file_seperator << delta_time << file_seperator << (delta_time*script_counter) << file_seperator
            << (FM1_8.velocity(rpm)*6) << file_seperator << (FM2_9.velocity(rpm)*6) << file_seperator
            << Target_Flywheel1_RPM << file_seperator << Target_Flywheel2_RPM << file_seperator
            << (Motor1_Error*Motor1_Kp) << file_seperator << (Motor1_Integral*Motor1_Ki) << file_seperator
            << (Motor1_Derivative*Motor1_Kd) << file_seperator << FM1_8_sentVoltage << file_seperator
            << Motor1_Integral_Limit << file_seperator 
            << (Motor2_Error*Motor2_Kp) << file_seperator << (Motor2_Integral*Motor2_Ki) << file_seperator 
            << (Motor2_Derivative*Motor2_Kd) << file_seperator << FM2_9_sentVoltage << file_seperator 
            << Motor2_Integral_Limit << file_seperator
            << FM1_8.current() << file_seperator << FM1_8.voltage() << file_seperator
            << FM1_8.power() << file_seperator << FM1_8.torque() << file_seperator
            << FM2_9.current() << file_seperator << FM2_9.voltage() << file_seperator
            << FM2_9.power() << file_seperator << FM2_9.torque() << file_seperator
            << FM1_8.efficiency() << file_seperator << FM1_8.temperature(celsius) << file_seperator  
            << FM2_9.efficiency() << file_seperator << FM2_9.temperature(celsius);  
    qLog.push(File_text.str());
    delta_time = Brain.timer(sec) - start_time;  
  }
  if (Button_Pressed)
  {
     FM1_8.setStopping(coast);
     FM2_9.setStopping(coast);
     FM1_8.stop();
     FM1_8.stop();
     write_file_from_queue(qLog);
  }  
}