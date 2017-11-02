#ifndef SERVOS_CONTROL_H
#define SERVOS_CONTROL_H

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include <ros/ros.h>
#include <signal.h>
#include "dynamixel_sdk/dynamixel_sdk.h"                                 // Uses Dynamixel SDK library
#include "dynamixel_servos/InfoMessage.h"
#include "dynamixel_servos/CommandMessage.h"

// Control table address

// EEPROM
#define ADDR_PRO_drive_mode            10 
#define ADDR_PRO_operating_mode        11   
#define ADDR_PRO_secondary_id          12   
#define ADDR_PRO_protocol_version      13   
#define ADDR_PRO_homing_offset         20   
#define ADDR_PRO_moving_threshold      24   
#define ADDR_PRO_temperature_limit     31  
#define ADDR_PRO_max_voltage_limit     32   
#define ADDR_PRO_min_voltage_limit     34   
#define ADDR_PRO_pwm_limit             36   
#define ADDR_PRO_current_limit         38  
#define ADDR_PRO_acceleration_limit    40
#define ADDR_PRO_velocity_limit        44   
#define ADDR_PRO_max_position_limit    48   
#define ADDR_PRO_min_position_limit    52  
#define ADDR_PRO_shutdown              63

//RAM
#define ADDR_PRO_torque_enable             64 
#define ADDR_PRO_led                       65      
#define ADDR_PRO_status_return_level       68    
#define ADDR_PRO_registered_instruction    69      
#define ADDR_PRO_hardware_error_status     70    
#define ADDR_PRO_velocity_i_gain           76     
#define ADDR_PRO_velocity_p_gain           78    
#define ADDR_PRO_position_d_gain           80      
#define ADDR_PRO_position_i_gain           82    
#define ADDR_PRO_position_p_gain           84     
#define ADDR_PRO_feedforward_2nd_gain      88    
#define ADDR_PRO_feedforward_1st_gain      90     
#define ADDR_PRO_bus_watchdog              98     
#define ADDR_PRO_goal_pwm                  100    
#define ADDR_PRO_goal_current              102    
#define ADDR_PRO_goal_velocity             104   
#define ADDR_PRO_profile_acceleration      108    
#define ADDR_PRO_profile_velocity          112    
#define ADDR_PRO_goal_position             116    
#define ADDR_PRO_realtime_tick             120    
#define ADDR_PRO_moving                    122      
#define ADDR_PRO_moving_status             123   
#define ADDR_PRO_present_pwm               124  
#define ADDR_PRO_present_current           126   
#define ADDR_PRO_present_velocity          128
#define ADDR_PRO_present_position          132
#define ADDR_PRO_velocity_trajectory       136
#define ADDR_PRO_position_trajectory       140     
#define ADDR_PRO_present_input_voltage     144    
#define ADDR_PRO_present_temperature       146 

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        3000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
#define FIRST_ID						21
#define SERVOS_NUMBER					3

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      500             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      2000             // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b



enum Commands
{
	disableTorque = 0,
	enableTorque = 1,
	writeGoalPosition = 2
};


class ServoControl
{
 private:

  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;

 public:
  ServoControl();
  ~ServoControl();

 private:

  int getch(void);
  int kbhit(void);
  
  public:
  
  bool open_port();
  bool set_baudrate();
  bool close_port();
  
  void enable_torque(uint8_t servo_id);
  void disable_torque(uint8_t servo_id);
  void write_goal_position(uint8_t servo_id, uint32_t goal_position);
  void write_register(uint8_t servo_id, uint8_t address, uint8_t bytes_number, uint32_t value);
  
  uint32_t read_present_position(uint8_t servo_id);
  uint32_t read_present_velocity(uint8_t servo_id);
  uint16_t read_present_current(uint8_t servo_id);
  
  void command_callback(const dynamixel_servos::CommandMessage::ConstPtr& msg);

};

#endif // SERVOS_CONTROL_H
