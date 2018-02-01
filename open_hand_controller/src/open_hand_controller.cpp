#include "ros/ros.h"
#include "open_hand_controller/contr_to_ros.h"
#include "open_hand_controller/ros_to_contr.h"
#include "open_hand_controller/close_hand.h"
#include "dynamixel_servos/CommandMessage.h"
#include "dynamixel_servos/InfoMessage.h"
#include <iostream>



#define servo1ID 21  // First finger
#define servo2ID 22  // Seccond finger
#define servo3ID 23  // Third finger
#define servo4ID 24  // Fingers rotation
/*
#define servo1ID 24  // First finger
#define servo2ID 23  // Seccond finger
#define servo3ID 22  // Third finger
#define servo4ID 21  // Fingers rotation
*/

#define posWirstMul 0.001047197
#define posFingersMul 0.001762977
#define velMul 0.02398
#define torqMul 0.0008382

#define pos1Bias 0
#define pos2Bias 0
#define pos3Bias 1.9
#define pos4Bias 0.88

/*
wersja z pojedynczym ograniczeniem
#define posMax 3.14
#define posMin 0*/

/*
Wersja z osobnymi ograniczeniami dla kazdego serwa*/
#define pos1Max 3.14 //1240
#define pos1Min 0
#define pos2Max 3.14
#define pos2Min 0
#define pos3Max 3.14
#define pos3Min 0
#define pos4Max 1.6 //1600
#define pos4Min 0


#define open_position 0
#define close_position 1.8

using namespace std;

void receive_msg_from_ros(const open_hand_controller::ros_to_contr& msg);
void receive_msg_from_servo(const dynamixel_servos::InfoMessage& msg);
void prepare_msg_to_ros(open_hand_controller::contr_to_ros& msg);
void receive_msg_close_hand(const open_hand_controller::close_hand& msg);


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
    void change_mode(int value);
    void change_velocity(int value);
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
        /*wersja z pojedynczym ograniczeniem
        if (value > posMax)
                        this->position_to_change = posMax;
                else if (value < posMin)
                        this->position_to_change = posMin;
                else
        this->position_to_change=value;*/

        /*Wersja z osobnymi ograniczeniami dla kazdego serwa */

        float posMax = 0;
                float posMin = 0;
                float posBias = 0;
                float posMul = 0;

                switch(this->id)
                {
                        case servo1ID:
                        {
                                posMax = pos1Max;
                                posMin = pos1Min;
                                posBias = pos1Bias;
                                posMul = posFingersMul;
                                break;
                        }

                        case servo2ID:
                        {
                                posMax = pos2Max;
                                posMin = pos2Min;
                                posBias = pos2Bias;
                                posMul = posFingersMul;
                                break;
                        }

                        case servo3ID:
                        {
                                posMax = pos3Max;
                                posMin = pos3Min;
                                posBias = pos3Bias;
                                posMul = -posFingersMul;
                                break;
                        }

                        case servo4ID:
                        {
                                posMax = pos4Max;
                                posMin = pos4Min;
                                posBias = pos4Bias;
                                posMul = posWirstMul;
                                break;
                        }
                }

                cout<<posMax<<" xx "<<posMin<<endl;

                if (value > posMax)
                        this->position_to_change = posMax;
                else if (value < posMin)
                        this->position_to_change = posMin;
                else
        this->position_to_change=value;




        this->position_to_change = value;
        dynamixel_servos::CommandMessage CommandMessage;
        CommandMessage.servo_id = this->id;

        CommandMessage.register_address = 112;
        CommandMessage.bytes_number = 4;
        CommandMessage.value = 20;

        this->publisher_to_servo.publish(CommandMessage);


        CommandMessage.register_address = 116;
        CommandMessage.bytes_number = 4;
        CommandMessage.value = (int)((this->position_to_change - posBias) / posMul);
        publisher_to_servo.publish(CommandMessage);
    }
}
void dynamixelServo::set_torque_to_change(float value)
{
    if(this->enable)
    {
        if (value > 1)
                        this->torque_to_change = 1;
                else if (value < 0)
                        this->torque_to_change = 0;
                else
        this->torque_to_change=value;

        dynamixel_servos::CommandMessage CommandMessage;
        CommandMessage.servo_id = this->id;
        CommandMessage.register_address = 102;
        CommandMessage.bytes_number = 2;
        CommandMessage.value = (int)(this->torque_to_change / torqMul);
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
    float posBias = 0;
    float posMul = 0;

    switch(this->id)
    {
            case servo1ID:
            {
                    posBias = pos1Bias;
                    posMul = posFingersMul;
                    break;
            }

            case servo2ID:
            {
                    posBias = pos2Bias;
                    posMul = posFingersMul;
                    break;
            }

            case servo3ID:
            {
                    posBias = pos3Bias;
                    posMul = -posFingersMul;
                    break;
            }

            case servo4ID:
            {
                    posBias = pos4Bias;
                    posMul = posWirstMul;
                    break;
            }
    }
    this->actual_position = value * posMul + posBias;
    //cout<<"aktualna pozycja:"<<id<<" "<<value<<endl;
    if(id==24) cout<<endl;
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
void dynamixelServo::change_mode(int value)
{
    dynamixel_servos::CommandMessage CommandMessage;
    CommandMessage.servo_id = this->id;
    CommandMessage.register_address = 11;
    CommandMessage.bytes_number = 1;
    CommandMessage.value = value;

    publisher_to_servo.publish(CommandMessage);
}
void dynamixelServo::change_velocity(int value)
{
    cout<<"wyslono predkosc"<<endl;
    dynamixel_servos::CommandMessage CommandMessage;
    CommandMessage.servo_id = this->id;
    CommandMessage.register_address = 112;
    CommandMessage.bytes_number = 4;
    CommandMessage.value = value;

    this->publisher_to_servo.publish(CommandMessage);
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
    ros::NodeHandle node_close_hand;

    ros::Publisher publisher_to_servo = node_to_servo.advertise<dynamixel_servos::CommandMessage>("servo_control_commands", 1000);
    ros::Subscriber subscriber_from_servo = node_from_servo.subscribe("servo_control_info", 1000, receive_msg_from_servo);

    ros::Publisher publisher_to_ros = node_to_ros.advertise<open_hand_controller::contr_to_ros>("contr_to_ros", 1000);
    ros::Subscriber subscriber_from_ros = node_from_ros.subscribe("ros_to_contr", 1000, receive_msg_from_ros);

    ros::Subscriber subscriber_close_hand = node_close_hand.subscribe("close_hand", 1000, receive_msg_close_hand);

    for(int i=0;i<4;i++)
    {
        Servo[i].set_publisher_to_servo(publisher_to_servo);
        //Servo[i].change_mode(5);
       // Servo[i].change_velocity(20);
    }


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
    if(msg.Finger1Enable) Servo[0].activate_servo();
    else Servo[0].deactivate_servo();

    if(msg.Finger2Enable) Servo[1].activate_servo();
    else Servo[1].deactivate_servo();

    if(msg.Finger3Enable) Servo[2].activate_servo();
    else Servo[2].deactivate_servo();

    if(msg.FingersRotationEnable) Servo[3].activate_servo();
    else Servo[3].deactivate_servo();


    for(int i=0;i<4;i++)
    {
        Servo[i].change_velocity(20);
    }

    Servo[0].set_position_to_change(msg.Finger1Position);
    Servo[1].set_position_to_change(msg.Finger2Position);
    Servo[2].set_position_to_change(msg.Finger3Position);
    Servo[3].set_position_to_change(msg.FingersRotationPosition);

    Servo[0].set_torque_to_change((float)msg.Finger1Torque);
    Servo[1].set_torque_to_change((float)msg.Finger2Torque);
    Servo[2].set_torque_to_change((float)msg.Finger3Torque);
    Servo[3].set_torque_to_change((float)msg.FingersRotationTorque);

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
    msg.Finger1Position = Servo[0].get_actual_position();
    msg.Finger2Position = Servo[1].get_actual_position();
    msg.Finger3Position = Servo[2].get_actual_position();
    msg.FingersRotationPosition = Servo[3].get_actual_position();

    msg.Finger1Velocity = Servo[0].get_actual_velocity();
    msg.Finger2Velocity = Servo[1].get_actual_velocity();
    msg.Finger3Velocity = Servo[2].get_actual_velocity();
    msg.FingersRotationVelocity = Servo[3].get_actual_velocity();

    msg.Finger1Torque = Servo[0].get_actual_torque();
    msg.Finger2Torque = Servo[1].get_actual_torque();
    msg.Finger3Torque = Servo[2].get_actual_torque();
    msg.FingersRotationTorque = Servo[3].get_actual_torque();
}

void receive_msg_close_hand(const open_hand_controller::close_hand& msg)
{
    if(msg.FingersClose)
    {
        for(int i=0;i<3;i++)
        {
            Servo[i].activate_servo();
            Servo[i].set_torque_to_change((float)msg.FingersTorque);
            Servo[i].set_position_to_change(close_position);
        }
    }

    if(!msg.FingersClose)
    {
        for(int i=0;i<3;i++)
        {
            Servo[i].activate_servo();
            Servo[i].set_torque_to_change((float)msg.FingersTorque);
            Servo[i].set_position_to_change(open_position);
        }
    }
}

