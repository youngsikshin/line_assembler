#include <line_assembler/LineMap.h>

#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace line_assembler {

LineMap::LineMap()
    : init_(false)
{

}

void LineMap::reset()
{

}

void LineMap::add_lines(std::vector<line_extractor::LineSegment>& lines)
{
    line_segments_.reserve(line_segments_.size()+lines.size());
    line_segments_.insert(line_segments_.end(), lines.begin(), lines.end());

    if(!init_) {
        if(line_segments_.size() > 0)
            init_ = true;
    } 
}

}