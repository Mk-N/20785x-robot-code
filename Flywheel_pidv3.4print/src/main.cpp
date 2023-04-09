/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Mukunth Natarajan @20875X                                 */
/*    Created:      Fri Feb 24 2023                                           */
/*    Description:  Performance boost: Even shorter time to calculate integral*/
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Flywheel_MotorP6N    motor         8               
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
double start_time                                  = 0;
double delta_time                                  = 0;
ostringstream File_text;
queue <string> qLog;

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

                if (f_FM_Coff_Frequency > 1)
                {
                  Flywheel_Motor_Cutoff_Frequency = 1;
                }
                else if (f_FM_Coff_Frequency < 0)
                {
                  Flywheel_Motor_Cutoff_Frequency = 0;
                }
                else
                {
                  Flywheel_Motor_Cutoff_Frequency = f_FM_Coff_Frequency;
                }

                if (f_FM_derCoff_Frequency > 1)
                {
                  Flywheel_Motor_Derivative_Cutoff_Frequency = 1;
                }
                else if (f_FM_derCoff_Frequency < 0)
                {
                  Flywheel_Motor_Derivative_Cutoff_Frequency = 0;
                }
                else
                {
                  Flywheel_Motor_Derivative_Cutoff_Frequency = f_FM_derCoff_Frequency;
                }
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
  if ((Flywheel_Motor_Error == 0) || (fabs(Flywheel_Motor_Error) > Flywheel_Motor_Integral_Limit) ||
     ((Target_Flywheel_Motor_RPM > 0) && (Previous_Flywheel_Motor_error > 0) && (Flywheel_Motor_Error < 0)) || 
     ((Target_Flywheel_Motor_RPM < 0) && (Previous_Flywheel_Motor_error < 0) && (Flywheel_Motor_Error > 0)))
  {
    Flywheel_Motor_Integral = 0;
  }   
  else 
  {
    Flywheel_Motor_Integral = Flywheel_Motor_Integral + Flywheel_Motor_Error * delta_time;
  }
  Flywheel_Motor_Filtered_Derivative = Flywheel_Motor_Derivative_Cutoff_Frequency 
                                       * ((Flywheel_Motor_Filtered_Velocity - Previous_Flywheel_Motor_Filtered_Velocity)/delta_time)
                                       + (1 - Flywheel_Motor_Derivative_Cutoff_Frequency) * Previous_Flywheel_Motor_Filtered_Derivative;
}

void set_motor_volt()
{
  if (Target_Flywheel_Motor_RPM > 0)
  {
    Flywheel_MotorP6N_sentVoltage = (Flywheel_Motor_Feedforwarded_Velocity + Flywheel_Motor_Kp * Flywheel_Motor_Error + Flywheel_Motor_Ki * Flywheel_Motor_Integral - Flywheel_Motor_Kd * Flywheel_Motor_Filtered_Derivative);
    Flywheel_MotorP6N.spin(directionType::fwd, Flywheel_MotorP6N_sentVoltage, volt);
  }
  else if (Target_Flywheel_Motor_RPM == 0) 
  {
    Flywheel_MotorP6N.setStopping(hold);
    Flywheel_MotorP6N.stop();
    Flywheel_MotorP6N.setStopping(coast);
  }
  else 
  {
    Flywheel_MotorP6N_sentVoltage = (Flywheel_Motor_Feedforwarded_Velocity + Flywheel_Motor_Kp * Flywheel_Motor_Error + Flywheel_Motor_Ki * Flywheel_Motor_Integral - Flywheel_Motor_Kd * Flywheel_Motor_Filtered_Derivative);
    Flywheel_MotorP6N.spin(directionType::fwd, Flywheel_MotorP6N_sentVoltage, volt);
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
  get_and_filter_motor_velocity(Flywheel_MotorP6N.velocity(rpm));
  set_motor_error();
  process_speed();
  set_motor_volt();
  record_previous_values();
}

void set_button_pressed_true()
{
  Button_Pressed = true;
}

void write_file_from_queue(queue<string> q)
{
  if (!Brain.SDcard.isInserted())
  {
    cout << "Please insert the sdcard. Try again." << endl;
    Brain.Screen.print("Please insert the sdcard. Try again.");
    return;
  }
  ofstream file_handler("result_log.csv", ofstream::out);
  while (!q.empty())
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
  // Target_Flywheel_Motor_RPM is between 0 and 3600
  // Keep them equal unless you are doing a double flywheel or require fine control
  set_values(Target_Flywheel_Motor_RPM, Flywheel_Motor_Kp, Flywheel_Motor_Ki, Flywheel_Motor_Kd, Flywheel_Motor_Integral_Limit, 
             Flywheel_Motor_Cutoff_Frequency, Flywheel_Motor_Derivative_Cutoff_Frequency, Flywheel_Motor_Kf);
  File_text << "Script counter,Delta time/seconds,Time elapsed/seconds,"
               "Flywheel motor RPM,Target Flywheel motor RPM,Flywheel motor filtered velocity,"
               "Flywheel motor proportional gain,Flywheel motor integral gain,Flywheel motor derivative gain,Flywheel motor feedforwarded gain,"
               "Flywheel motor commanded voltage/volts,Flywheel motor integral limit,"
               "Flywheel motor current/amperes,Flywheel motor voltage/volts,Flywheel motor power/watts,Flywheel motor torque/newton meters,"
               "Flywheel motor efficiency/pct,Flywheel motor temperature/celsius,"
               "Flywheel motor Kp,Flywheel motor Ki,Flywheel motor Kd,Flywheel motor Kf,"
               "Flywheel motor cutoff frequency,Flywheel motor derivative cutoff frequency";
  qLog.push(File_text.str());
  wait(500, msec);
  Brain.resetTimer();
  start_time = 0;
  get_and_filter_motor_velocity(Flywheel_MotorP6N.velocity(rpm));
  set_motor_error();
  set_motor_volt();
  record_previous_values();
  Script_Counter = 1;
  File_text << Script_Counter << File_Seperator << Brain.timer(sec) << File_Seperator << Brain.timer(sec) << File_Seperator
            << (Flywheel_MotorP6N.velocity(rpm)*6) << File_Seperator 
            << Target_Flywheel_Motor_RPM << File_Seperator << (Flywheel_Motor_Filtered_Velocity*6) << File_Seperator
            << (Flywheel_Motor_Error*Flywheel_Motor_Kp) << File_Seperator << (Flywheel_Motor_Integral*Flywheel_Motor_Ki) << File_Seperator
            << (-Flywheel_Motor_Filtered_Derivative*Flywheel_Motor_Kd) << File_Seperator << Flywheel_Motor_Feedforwarded_Velocity << File_Seperator
            << Flywheel_MotorP6N_sentVoltage << File_Seperator << Flywheel_Motor_Integral_Limit << File_Seperator 
            << Flywheel_MotorP6N.current() << File_Seperator << Flywheel_MotorP6N.voltage() << File_Seperator
            << Flywheel_MotorP6N.power() << File_Seperator << Flywheel_MotorP6N.torque() << File_Seperator
            << Flywheel_MotorP6N.efficiency() << File_Seperator << Flywheel_MotorP6N.temperature(celsius) << File_Seperator  
            << Flywheel_Motor_Kp << File_Seperator << Flywheel_Motor_Ki << File_Seperator << Flywheel_Motor_Kd << Flywheel_Motor_Kf << File_Seperator
            << Flywheel_Motor_Cutoff_Frequency << File_Seperator << Flywheel_Motor_Derivative_Cutoff_Frequency << File_Seperator;  
  qLog.push(File_text.str());
  delta_time = Brain.timer(sec);

  while (!Button_Pressed)
  {
    delta_time = Brain.timer(sec) - start_time;
    start_time = Brain.timer(sec);
    flywheel_pid();
    Script_Counter++;
    File_text << Script_Counter << File_Seperator << delta_time << File_Seperator << (delta_time * Script_Counter) << File_Seperator
              << (Flywheel_MotorP6N.velocity(rpm)*6) << File_Seperator 
              << Target_Flywheel_Motor_RPM << File_Seperator << (Flywheel_Motor_Filtered_Velocity*6) << File_Seperator
              << (Flywheel_Motor_Error*Flywheel_Motor_Kp) << File_Seperator << (Flywheel_Motor_Integral*Flywheel_Motor_Ki) << File_Seperator
              << (-Flywheel_Motor_Filtered_Derivative*Flywheel_Motor_Kd) << File_Seperator << Flywheel_Motor_Feedforwarded_Velocity << File_Seperator
              << Flywheel_MotorP6N_sentVoltage << File_Seperator << Flywheel_Motor_Integral_Limit << File_Seperator 
              << Flywheel_MotorP6N.current() << File_Seperator << Flywheel_MotorP6N.voltage() << File_Seperator
              << Flywheel_MotorP6N.power() << File_Seperator << Flywheel_MotorP6N.torque() << File_Seperator
              << Flywheel_MotorP6N.efficiency() << File_Seperator << Flywheel_MotorP6N.temperature(celsius) << File_Seperator  
              << Flywheel_Motor_Kp << File_Seperator << Flywheel_Motor_Ki << File_Seperator << Flywheel_Motor_Kd << Flywheel_Motor_Kf << File_Seperator
              << Flywheel_Motor_Cutoff_Frequency << File_Seperator << Flywheel_Motor_Derivative_Cutoff_Frequency << File_Seperator;  
    qLog.push(File_text.str());  
  }
  delta_time = Brain.timer(sec);
  if (Button_Pressed)
  {
    Flywheel_MotorP6N.setStopping(coast);
    Flywheel_MotorP6N.stop();
    write_file_from_queue(qLog);
  }
  return 1;  
}