#include "ros/ros.h"
#include "open_hand_controller/ros_to_contr.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tes_talker");

    ros::NodeHandle node_to_contr;

    ros::Publisher publisher_to_contr = node_to_contr.advertise<open_hand_controller::ros_to_contr>("ros_to_contr", 1000);

    open_hand_controller::ros_to_contr msg;

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        printf("\033c");
        char enable;
        float value;

        cout<<"Servo 1:"<<endl<<"Position:\t";
        cin>>value;
        msg.Finger1Position=value;

        cout<<"Torque:\t\t";
        cin>>value;
        msg.Finger1Torque=value;

        cout<<"Enable(t/n):\t";
        cin>>enable;
        if(enable=='t' || enable=='T') msg.Finger1Enable=true;
        else msg.Finger1Enable=false;

        cout<<endl<<endl<<"Servo 2:"<<endl<<"Position:\t";
        cin>>value;
        msg.Finger2Position=value;

        cout<<"Torque:\t\t";
        cin>>value;
        msg.Finger2Torque=value;

        cout<<"Enable(t/n):\t";
        cin>>enable;
        if(enable=='t' || enable=='T') msg.Finger2Enable=true;
        else msg.Finger2Enable=false;

        cout<<endl<<endl<<"Servo 3:"<<endl<<"Position:\t";
        cin>>value;
        msg.Finger3Position=value;

        cout<<"Torque:\t\t";
        cin>>value;
        msg.Finger3Torque=value;

        cout<<"Enable(t/n):\t";
        cin>>enable;
        if(enable=='t' || enable=='T') msg.Finger3Enable=true;
        else msg.Finger3Enable=false;

        cout<<endl<<endl<<"Servo 4:"<<endl<<"Position:\t";
        cin>>value;
        msg.FingersRotationPosition=value;

        cout<<"Torque:\t\t";
        cin>>value;
        msg.FingersRotationTorque=value;

        cout<<"Enable(t/n):\t";
        cin>>enable;
        if(enable=='t' || enable=='T') msg.FingersRotationEnable=true;
        else msg.FingersRotationEnable=false;

        publisher_to_contr.publish(msg);
        loop_rate.sleep();
    }


    return 0;
}
