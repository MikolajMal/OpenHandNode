#include "ros/ros.h"
#include "open_hand_controller/contr_to_ros.h"
#include <iostream>
#include <stdlib.h>

using namespace std;


void receive_msg_from_contr(const open_hand_controller::contr_to_ros& msg);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_listener");

    ros::NodeHandle node_from_contr;

    ros::Subscriber subscriber_from_contr = node_from_contr.subscribe("contr_to_ros", 1000, receive_msg_from_contr);

    ros::spin();
}


void receive_msg_from_contr(const open_hand_controller::contr_to_ros& msg)
{
    printf("\033c");
    cout << setprecision(2) << fixed;
    cout<<"Servo 1:"<<endl<<"Position:\t"<<(float)msg.Finger1Position<<endl<<"Velocity:\t"<<(float)msg.Finger1Velocity<<endl<<"Torque:\t\t"<<msg.Finger1Torque<<endl<<endl;
    cout<<"Servo 2:"<<endl<<"Position:\t"<<(float)msg.Finger2Position<<endl<<"Velocity:\t"<<(float)msg.Finger2Velocity<<endl<<"Torque:\t\t"<<msg.Finger2Torque<<endl<<endl;
    cout<<"Servo 3:"<<endl<<"Position:\t"<<(float)msg.Finger3Position<<endl<<"Velocity:\t"<<(float)msg.Finger3Velocity<<endl<<"Torque:\t\t"<<msg.Finger3Torque<<endl<<endl;
    cout<<"Servo 4:"<<endl<<"Position:\t"<<(float)msg.FingersRotationPosition<<endl<<"Velocity:\t"<<(float)msg.FingersRotationVelocity<<endl<<"Torque:\t\t"<<msg.FingersRotationTorque<<endl<<endl;
}
