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
        msg.Position1=value;

        cout<<"Torque:\t\t";
        cin>>value;
        msg.Torque1=value;

        cout<<"Enable(t/n):\t";
        cin>>enable;
        if(enable=='t' || enable=='T') msg.enable1=true;
        else msg.enable1=false;

        cout<<endl<<endl<<"Servo 2:"<<endl<<"Position:\t";
        cin>>value;
        msg.Position2=value;

        cout<<"Torque:\t\t";
        cin>>value;
        msg.Torque2=value;

        cout<<"Enable(t/n):\t";
        cin>>enable;
        if(enable=='t' || enable=='T') msg.enable2=true;
        else msg.enable2=false;

        cout<<endl<<endl<<"Servo 3:"<<endl<<"Position:\t";
        cin>>value;
        msg.Position3=value;

        cout<<"Torque:\t\t";
        cin>>value;
        msg.Torque3=value;

        cout<<"Enable(t/n):\t";
        cin>>enable;
        if(enable=='t' || enable=='T') msg.enable3=true;
        else msg.enable3=false;

        cout<<endl<<endl<<"Servo 4:"<<endl<<"Position:\t";
        cin>>value;
        msg.Position4=value;

        cout<<"Torque:\t\t";
        cin>>value;
        msg.Torque4=value;

        cout<<"Enable(t/n):\t";
        cin>>enable;
        if(enable=='t' || enable=='T') msg.enable4=true;
        else msg.enable4=false;

        publisher_to_contr.publish(msg);
        loop_rate.sleep();
    }


    return 0;
}
