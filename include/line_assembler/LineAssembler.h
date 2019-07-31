#include <iostream>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <line_extractor/LineListMessage.h>
#include <line_extractor/LineSegment.h>

#include <line_assembler/LineMap.h>
#include <line_assembler/LineMatcher.h>
#include <line_assembler/LineFactor.hpp>
#include <line_assembler/Pose2d.h>

using namespace std;

namespace line_assembler
{

class LineAssembler
{
public:
    LineAssembler(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~LineAssembler() { }
    void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void lines_callback(const line_extractor::LineListMessage::ConstPtr& lines_msg);
    void reset();

private:
    ros::Publisher scan_marker_pub_;
    ros::Publisher map_marker_pub_;
    
    ros::Subscriber odom_sub_;
    ros::Subscriber lines_sub_;
    std::vector<line_extractor::LineSegment> line_segments_;
    std::vector<line_extractor::LineSegment> local_line_segments_;

    Pose2d pose_;
    Pose2d prev_odom_;

    Eigen::Isometry2f T_prev_cur_;

    double params[3] = {0.0, 0.0, 0.0};

    LineMap::Ptr map_;
    LineMatcher::Ptr matcher_;
};

}