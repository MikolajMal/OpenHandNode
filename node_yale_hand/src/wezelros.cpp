#include "ros/ros.h"
#include "node_yale_hand/s2ros.h"
#include "node_yale_hand/ros2s.h"
#include "dynamixel_servos/CommandMessage.h"
#include "dynamixel_servos/InfoMessage.h"
#include <iostream>


#define servo1ID 1
#define servo2ID 2
#define servo3ID 3
#define servo4ID 4

#define posMul 1
#define velMul 1
#define torqMul 1



using namespace std;







void odRos(const node_yale_hand::ros2s& msg);
void odSerwo(const dynamixel_servos::InfoMessage& msg);
void przygotujWiadomoscDoRos(node_yale_hand::s2ros& msg);












class dynamixelServo
{
private:
    int id;
    float actPosition;
    float actVelocity;
    float actTorque;
    float setPosition;
    float setTorque;
    bool torque;

public:
    dynamixelServo();
    void SetID(int value);
    void SetPosition(float value, ros::Publisher& pub);
    void SetTorque(float value);
    void ActivateTorque(ros::Publisher &pub);
    void DeactivateTorque(ros::Publisher &pub);
    void SetActPosition(int value);
    void SetActVelocity(int value);
    void SetActTorque(int value);

    float GetActPosition();
    float GetActVelocity();
    float GetActTorque();
};

dynamixelServo::dynamixelServo()
{
   actPosition=0;
   actTorque=0;
   actVelocity=0;
   torque=false;
}
void dynamixelServo::SetID(int value)
{
    this->id=value;
}
void dynamixelServo::SetPosition(float value, ros::Publisher &pub)
{

    this->setPosition = value;

    dynamixel_servos::CommandMessage CommandMessage;

    CommandMessage.servo_id = this->id;
    CommandMessage.register_address = 116;
    CommandMessage.bytes_number = 4;
    CommandMessage.value = value / posMul;

    pub.publish(CommandMessage);
}
void dynamixelServo::SetTorque(float value)
{
   actTorque=value;
}
void dynamixelServo::ActivateTorque(ros::Publisher &pub)
{
    this->torque = true;

    dynamixel_servos::CommandMessage CommandMessage;

    CommandMessage.servo_id = this->id;
    CommandMessage.register_address = 64;
    CommandMessage.bytes_number = 1;
    CommandMessage.value = 1;

    pub.publish(CommandMessage);
}
void dynamixelServo::DeactivateTorque(ros::Publisher &pub)
{

    this->torque = false;

    dynamixel_servos::CommandMessage CommandMessage;

    CommandMessage.servo_id = this->id;
    CommandMessage.register_address = 64;
    CommandMessage.bytes_number = 1;
    CommandMessage.value = 0;

    pub.publish(CommandMessage);
}
void dynamixelServo::SetActPosition(int value)
{
    this->actPosition = value * posMul;
}
void dynamixelServo::SetActVelocity(int value)
{
    this->actVelocity = value * velMul;
}
void dynamixelServo::SetActTorque(int value)
{
    this->actTorque = value * torqMul;
}
float dynamixelServo::GetActPosition()
{
    return this->actPosition;
}
float dynamixelServo::GetActVelocity()
{
    return this->actVelocity;
}
float dynamixelServo::GetActTorque()
{
    return this->actTorque;
}











dynamixelServo Servo[4];
ros::NodeHandle n1;
ros::NodeHandle n2;
ros::NodeHandle n3;
ros::NodeHandle n4;
ros::Publisher serwoPub = n1.advertise<dynamixel_servos::CommandMessage>("servo_control_commands", 1000);
ros::Subscriber serwoSub = n2.subscribe("servo_control_info", 1000, odSerwo);
ros::Publisher rosPub = n3.advertise<node_yale_hand::s2ros>("t_w2ros", 1000);
ros::Subscriber rosSub = n4.subscribe("t_ros2w", 1000, odRos);

int main(int argc, char **argv)
{
    Servo[0].SetID(servo1ID);
    Servo[1].SetID(servo2ID);
    Servo[2].SetID(servo3ID);
    Servo[3].SetID(servo4ID);

    ros::init(argc, argv, "wezelros");

    ros::Rate loop_rate(10);

    node_yale_hand::s2ros servoMessage;

    while(ros::ok())
    {        
        ros::spinOnce();

        przygotujWiadomoscDoRos(servoMessage);
        rosPub.publish(servoMessage);

        loop_rate.sleep();
    }
}

void odRos(const node_yale_hand::ros2s& msg)
{

    Servo[0].SetPosition(msg.Position1, serwoPub);
    Servo[1].SetPosition(msg.Position2, serwoPub);
    Servo[2].SetPosition(msg.Position3, serwoPub);
    Servo[3].SetPosition(msg.Position4, serwoPub);

    Servo[0].SetTorque(msg.Torque1);
    Servo[1].SetTorque(msg.Torque2);
    Servo[2].SetTorque(msg.Torque3);
    Servo[3].SetTorque(msg.Torque4);

    if(msg.enable1) Servo[0].ActivateTorque(serwoPub);
    else Servo[0].DeactivateTorque(serwoPub);

    if(msg.enable2) Servo[1].ActivateTorque(serwoPub);
    else Servo[1].DeactivateTorque(serwoPub);

    if(msg.enable3) Servo[2].ActivateTorque(serwoPub);
    else Servo[2].DeactivateTorque(serwoPub);

    if(msg.enable4) Servo[3].ActivateTorque(serwoPub);
    else Servo[3].DeactivateTorque(serwoPub);
}

void odSerwo(const dynamixel_servos::InfoMessage& msg)
 {

     switch(msg.servo_id)
     {
    // case servo1ID:
     case 1:
     {
         Servo[0].SetActPosition(msg.present_position);
         Servo[0].SetActVelocity(msg.present_velocity);
         Servo[0].SetActTorque(msg.present_current);
         break;
     }

     //case servo2ID:
     case 2:
     {
         Servo[1].SetActPosition(msg.present_position);
         Servo[1].SetActVelocity(msg.present_velocity);
         Servo[1].SetActTorque(msg.present_current);
         break;
     }

     //case servo3ID:
     case 3:
     {
         Servo[2].SetActPosition(msg.present_position);
         Servo[2].SetActVelocity(msg.present_velocity);
         Servo[2].SetActTorque(msg.present_current);
         break;
     }

     //case servo4ID:
     case 4:
     {
         Servo[3].SetActPosition(msg.present_position);
         Servo[3].SetActVelocity(msg.present_velocity);
         Servo[3].SetActTorque(msg.present_current);
         break;
     }

     default:
         cout << "Nieznane ID serwa" << endl;
         break;
     }
 }

void przygotujWiadomoscDoRos(node_yale_hand::s2ros& msg)
{
    msg.Position1 = Servo[0].GetActPosition();
    msg.Position2 = Servo[1].GetActPosition();
    msg.Position3 = Servo[2].GetActPosition();
    msg.Position4 = Servo[3].GetActPosition();

    msg.Velocity1 = Servo[0].GetActVelocity();
    msg.Velocity2 = Servo[1].GetActVelocity();
    msg.Velocity3 = Servo[2].GetActVelocity();
    msg.Velocity4 = Servo[3].GetActVelocity();

    msg.Torque1 = Servo[0].GetActTorque();
    msg.Torque2 = Servo[1].GetActTorque();
    msg.Torque3 = Servo[2].GetActTorque();
    msg.Torque4 = Servo[3].GetActTorque();
}

