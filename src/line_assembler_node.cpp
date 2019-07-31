#include <iostream>
#include <ros/ros.h>
#include <line_assembler/LineAssembler.h>

using namespace std;
using namespace line_assembler;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_assembler");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    LineAssembler assembler(nh, private_nh);

    ros::spin();
    
    return 0;
}