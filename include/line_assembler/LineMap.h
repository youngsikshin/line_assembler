#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <line_extractor/LineSegment.h>
#include <line_assembler/LineMatcher.h>

using namespace std;

namespace line_assembler
{

class LineMap : public LineMatcher
{
public:
    typedef shared_ptr<LineMap> Ptr;
    LineMap();
    ~LineMap() { }
    void reset();
    bool init() { return init_; }
    void add_line(line_extractor::LineSegment& line) { line_segments_.push_back(line); }
    void add_lines(std::vector<line_extractor::LineSegment>& lines);
    std::vector<line_extractor::LineSegment>& lines() { return line_segments_; }

private:
    bool init_;
    
    std::vector<line_extractor::LineSegment> line_segments_;
};

}