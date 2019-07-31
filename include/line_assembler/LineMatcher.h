#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <utility>
#include <unordered_map>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <line_extractor/LineSegment.h>

using namespace std;

namespace line_assembler
{

class LineMatcher
{
public:
    typedef shared_ptr<LineMatcher> Ptr;
    LineMatcher();
    ~LineMatcher() { }

    std::unordered_map<int, int> time_based_matching(std::vector<line_extractor::LineSegment>& scan_lines, std::vector<line_extractor::LineSegment>& map_lines);
    std::vector<int> find_subset_using_time(std::vector<line_extractor::LineSegment>& map_lines, std_msgs::Header header);
    std::vector<int> find_subset_using_range_limit(std::vector<line_extractor::LineSegment>& map_lines, float x, float y);

    std::unordered_map<int, int> find_matching_pair_from_vecslam(std::vector<line_extractor::LineSegment>& scan_lines, 
                                                                 std::vector<line_extractor::LineSegment>& map_lines, 
                                                                 std::vector<int>& map_idx);

    std::unordered_map<int, int> find_matching_pair_from_simple_distance(std::vector<line_extractor::LineSegment>& scan_lines, 
                                                                         std::vector<line_extractor::LineSegment>& map_lines, 
                                                                         std::vector<int>& map_idx);                                                             

    float find_min_quadratic_eq(float a, float b, float c);
    void reset();

private:
    float time_threshold_;
    float range_threshold_;

    float angle_threshold_;
};

}