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
    cout<<"Servo 1:"<<endl<<"Position:\t"<<(float)msg.Position1<<endl<<"Velocity:\t"<<(float)msg.Velocity1<<endl<<"Torque:\t\t"<<msg.Torque1<<endl<<endl;
    cout<<"Servo 2:"<<endl<<"Position:\t"<<(float)msg.Position2<<endl<<"Velocity:\t"<<(float)msg.Velocity2<<endl<<"Torque:\t\t"<<msg.Torque2<<endl<<endl;
    cout<<"Servo 3:"<<endl<<"Position:\t"<<(float)msg.Position3<<endl<<"Velocity:\t"<<(float)msg.Velocity3<<endl<<"Torque:\t\t"<<msg.Torque3<<endl<<endl;
    cout<<"Servo 4:"<<endl<<"Position:\t"<<(float)msg.Position4<<endl<<"Velocity:\t"<<(float)msg.Velocity4<<endl<<"Torque:\t\t"<<msg.Torque4<<endl<<endl;
}
