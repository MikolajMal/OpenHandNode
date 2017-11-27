#include "ros/ros.h"
#include "open_hand_controller/contr_to_ros.h"
#include "open_hand_controller/ros_to_contr.h"
#include "dynamixel_servos/CommandMessage.h"
#include "dynamixel_servos/InfoMessage.h"
#include <iostream>


#define servo1ID 21
#define servo2ID 22
#define servo3ID 23
#define servo4ID 24

#define posMul 0.00007773
#define posBias 0
#define velMul 0.02398
#define torqMul 0.0008382

using namespace std;

void receive_msg_from_ros(const open_hand_controller::ros_to_contr& msg);
void receive_msg_from_servo(const dynamixel_servos::InfoMessage& msg);
void prepare_msg_to_ros(open_hand_controller::contr_to_ros& msg);


class dynamixelServo
{
private:
    int id;
    float actual_position;
    float actual_velocity;
    float actual_torque;
    float position_to_change;
    float torque_to_change;
    bool enable;
    ros::Publisher publisher_to_servo;

public:
    dynamixelServo();
    void set_id(int value);
    void set_position_to_change(float value);
    void set_torque_to_change(float value);
    void activate_servo();
    void deactivate_servo();
    void set_actual_position(int value);
    void set_actual_velocity(int value);
    void set_actual_torque(int value);

    float get_actual_position();
    float get_actual_velocity();
    float get_actual_torque();

    void set_publisher_to_servo(ros::Publisher& handler);
};

dynamixelServo::dynamixelServo()
{
   actual_position=0;
   actual_torque=0;
   actual_velocity=0;
   enable=false;
}
void dynamixelServo::set_id(int value)
{
    this->id=value;
}
void dynamixelServo::set_position_to_change(float value)
{
    if(this->enable)
    {
        this->position_to_change = value;
        dynamixel_servos::CommandMessage CommandMessage;
        CommandMessage.servo_id = this->id;
        CommandMessage.register_address = 116;
        CommandMessage.bytes_number = 4;     
        CommandMessage.value = (int)((value - posBias) / posMul);
        publisher_to_servo.publish(CommandMessage);
    }
}
void dynamixelServo::set_torque_to_change(float value)
{
    if(this->enable)
    {
        this->torque_to_change=value;

        dynamixel_servos::CommandMessage CommandMessage;
        CommandMessage.servo_id = this->id;
        CommandMessage.register_address = 38;
        CommandMessage.bytes_number = 2;
        CommandMessage.value = (int)(value / torqMul);
        publisher_to_servo.publish(CommandMessage);
    }
}
void dynamixelServo::activate_servo()
{
    this->enable = true;
    dynamixel_servos::CommandMessage CommandMessage;
    CommandMessage.servo_id = this->id;
    CommandMessage.register_address = 64;
    CommandMessage.bytes_number = 1;
    CommandMessage.value = 1;
    publisher_to_servo.publish(CommandMessage);
}
void dynamixelServo::deactivate_servo()
{
    this->enable = false;
    dynamixel_servos::CommandMessage CommandMessage;
    CommandMessage.servo_id = this->id;
    CommandMessage.register_address = 64;
    CommandMessage.bytes_number = 1;
    CommandMessage.value = 0;

    publisher_to_servo.publish(CommandMessage);
}
void dynamixelServo::set_actual_position(int value)
{
    this->actual_position = value * posMul + posBias;
}
void dynamixelServo::set_actual_velocity(int value)
{
    this->actual_velocity = value * velMul;
}
void dynamixelServo::set_actual_torque(int value)
{
    this->actual_torque = value * torqMul;
}
float dynamixelServo::get_actual_position()
{
    return this->actual_position;
}
float dynamixelServo::get_actual_velocity()
{
    return this->actual_velocity;
}
float dynamixelServo::get_actual_torque()
{
    return this->actual_torque;
}
void dynamixelServo::set_publisher_to_servo(ros::Publisher& handler)
{
    this->publisher_to_servo = handler;
}


dynamixelServo Servo[4];

int main(int argc, char **argv)
{
    Servo[0].set_id(servo1ID);
    Servo[1].set_id(servo2ID);
    Servo[2].set_id(servo3ID);
    Servo[3].set_id(servo4ID);

    ros::init(argc, argv, "open_hand_controller");

    ros::NodeHandle node_to_servo;
    ros::NodeHandle node_from_servo;
    ros::NodeHandle node_to_ros;
    ros::NodeHandle node_from_ros;

    ros::Publisher publisher_to_servo = node_to_servo.advertise<dynamixel_servos::CommandMessage>("servo_control_commands", 1000);
    ros::Subscriber subscriber_from_servo = node_from_servo.subscribe("servo_control_info", 1000, receive_msg_from_servo);

    ros::Publisher publisher_to_ros = node_to_ros.advertise<open_hand_controller::contr_to_ros>("contr_to_ros", 1000);
    ros::Subscriber subscriber_from_ros = node_from_ros.subscribe("ros_to_contr", 1000, receive_msg_from_ros);

    for(int i=0;i<4;i++) Servo[i].set_publisher_to_servo(publisher_to_servo);

    ros::Rate loop_rate(10);

    open_hand_controller::contr_to_ros contr_to_ros_msg;

    while(ros::ok())
    {        
        ros::spinOnce();

        prepare_msg_to_ros(contr_to_ros_msg);
        publisher_to_ros.publish(contr_to_ros_msg);

        loop_rate.sleep();
    }
}

void receive_msg_from_ros(const open_hand_controller::ros_to_contr& msg)
{
    if(msg.enable1) Servo[0].activate_servo();
    else Servo[0].deactivate_servo();

    if(msg.enable2) Servo[1].activate_servo();
    else Servo[1].deactivate_servo();

    if(msg.enable3) Servo[2].activate_servo();
    else Servo[2].deactivate_servo();

    if(msg.enable4) Servo[3].activate_servo();
    else Servo[3].deactivate_servo();

    Servo[0].set_position_to_change(msg.Position1);
    Servo[1].set_position_to_change(msg.Position2);
    Servo[2].set_position_to_change(msg.Position3);
    Servo[3].set_position_to_change(msg.Position4);

    
    if (msg.Torque1 > 1)
        Servo[0].set_torque_to_change(1);
    else if (msg.Torque1 <0)
        Servo[0].set_torque_to_change(0);
    else
        Servo[0].set_torque_to_change(msg.Torque1);

    if (msg.Torque2 > 1)
        Servo[1].set_torque_to_change(1);
    else if (msg.Torque2 <0)
        Servo[1].set_torque_to_change(0);
    else
        Servo[1].set_torque_to_change(msg.Torque2);

    if (msg.Torque3 > 1)
        Servo[2].set_torque_to_change(1);
    else if (msg.Torque3 <0)
        Servo[2].set_torque_to_change(0);
    else
        Servo[2].set_torque_to_change(msg.Torque3);

    if (msg.Torque4 > 1)
        Servo[3].set_torque_to_change(1);
    else if (msg.Torque4 <0)
        Servo[3].set_torque_to_change(0);
    else
        Servo[3].set_torque_to_change(msg.Torque4);
}

void receive_msg_from_servo(const dynamixel_servos::InfoMessage& msg)
 {

     switch((int)msg.servo_id)
     {
     case servo1ID:
     {
         Servo[0].set_actual_position(msg.present_position);
         Servo[0].set_actual_velocity(msg.present_velocity);
         Servo[0].set_actual_torque(msg.present_current);
         break;
     }

     case servo2ID:
     {
         Servo[1].set_actual_position(msg.present_position);
         Servo[1].set_actual_velocity(msg.present_velocity);
         Servo[1].set_actual_torque(msg.present_current);
         break;
     }

     case servo3ID:
     {
         Servo[2].set_actual_position(msg.present_position);
         Servo[2].set_actual_velocity(msg.present_velocity);
         Servo[2].set_actual_torque(msg.present_current);
         break;
     }

     case servo4ID:
     {
         Servo[3].set_actual_position(msg.present_position);
         Servo[3].set_actual_velocity(msg.present_velocity);
         Servo[3].set_actual_torque(msg.present_current);
         break;
     }

     default:
         cout << "Nieznane ID serwa:" << (int)msg.servo_id<<endl;
         break;
     }
 }

void prepare_msg_to_ros(open_hand_controller::contr_to_ros& msg)
{
    msg.Position1 = Servo[0].get_actual_position();
    msg.Position2 = Servo[1].get_actual_position();
    msg.Position3 = Servo[2].get_actual_position();
    msg.Position4 = Servo[3].get_actual_position();

    msg.Velocity1 = Servo[0].get_actual_velocity();
    msg.Velocity2 = Servo[1].get_actual_velocity();
    msg.Velocity3 = Servo[2].get_actual_velocity();
    msg.Velocity4 = Servo[3].get_actual_velocity();

    msg.Torque1 = Servo[0].get_actual_torque();
    msg.Torque2 = Servo[1].get_actual_torque();
    msg.Torque3 = Servo[2].get_actual_torque();
    msg.Torque4 = Servo[3].get_actual_torque();
}

