#include <line_assembler/LineMatcher.h>

#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace line_assembler {

LineMatcher::LineMatcher()
    : time_threshold_(5.0), range_threshold_(10.0)
{
    angle_threshold_ = 10.0/180.0*M_PI;
}

std::unordered_map<int, int> LineMatcher::time_based_matching(std::vector<line_extractor::LineSegment>& scan_lines, std::vector<line_extractor::LineSegment>& map_lines)
{
    std_msgs::Header header = scan_lines[0].header();

    std::vector<int> map_idx = find_subset_using_time(map_lines, header);

    // return find_matching_pair_from_vecslam(scan_lines, map_lines, map_idx);
    return find_matching_pair_from_simple_distance(scan_lines, map_lines, map_idx);
}

std::vector<int> LineMatcher::find_subset_using_time(std::vector<line_extractor::LineSegment>& map_lines, std_msgs::Header header)
{
    std::vector<int> id_list;

    for(int i=0; i<map_lines.size(); ++i) {
        double time_diff = map_lines[i].get_time_diff(header);
        // std::cout << "time_diff : " << time_diff << std::endl;
        if(time_diff < time_threshold_) {
            id_list.push_back(i);
        }
    }
    
    return id_list;
}

std::vector<int> LineMatcher::find_subset_using_range_limit(std::vector<line_extractor::LineSegment>& map_lines, float x, float y)
{
    std::vector<int> id_list;

    for(int i=0; i<map_lines.size(); ++i) {
        float dist1 = map_lines[i].sp().calc_distance_from(line_extractor::Point(x,y));
        float dist2 = map_lines[i].ep().calc_distance_from(line_extractor::Point(x,y));

        if( dist1 < range_threshold_ || dist2 < range_threshold_) {
            id_list.push_back(i);
        }
    }
}

std::unordered_map<int, int> LineMatcher::find_matching_pair_from_vecslam(std::vector<line_extractor::LineSegment>& scan_lines, 
                                                                          std::vector<line_extractor::LineSegment>& map_lines, 
                                                                          std::vector<int>& map_idx)
{
    std::unordered_map<int, int> matching_pairs;

    for(int i=0; i < scan_lines.size(); ++i) {

        float min_dist = std::numeric_limits<float>::max();
        std::pair<int, int> matching_pair;
        
        for(int idx:map_idx) {
            line_extractor::LineSegment& scan_line = scan_lines[i];
            line_extractor::LineSegment& map_line = map_lines[idx];

            if( fabs(line_extractor::LineSegment::calc_angle(scan_line, map_line)) > fabs(angle_threshold_))
                continue;

            float a1 = map_line.length();
            float b1 = 2.0*( (scan_line.sp().x()-map_line.sp().x())*(map_line.ep().x()-map_line.sp().x()) 
                            + (scan_line.sp().y()-map_line.sp().y())*(map_line.ep().y()-map_line.sp().y()) );
            float c1 = sqrt( (scan_line.sp().x()-map_line.sp().x())*(scan_line.sp().x()-map_line.sp().x()) +
                                (scan_line.sp().y()-map_line.sp().y())*(scan_line.sp().y()-map_line.sp().y()) );

            float dist1 = find_min_quadratic_eq(a1, b1, c1);

            float a2 = map_line.length();
            float b2 = 2.0*( (scan_line.ep().x()-map_line.sp().x())*(map_line.ep().x()-map_line.sp().x()) 
                            + (scan_line.ep().y()-map_line.sp().y())*(map_line.ep().y()-map_line.sp().y()) );
            float c2 = sqrt( (scan_line.ep().x()-map_line.sp().x())*(scan_line.ep().x()-map_line.sp().x()) +
                                (scan_line.ep().y()-map_line.sp().y())*(scan_line.ep().y()-map_line.sp().y()) );
            // float b2 = -2.0*( (scan_line.ep().x()-map_line.ep().x())*(map_line.ep().x()-map_line.sp().x()) 
            //               + (scan_line.ep().y()-map_line.ep().y())*(map_line.ep().y()-map_line.sp().y()) );
            // float c2 = sqrt( (scan_line.ep().x()-map_line.ep().x())*(scan_line.ep().x()-map_line.ep().x()) +
            //                  (scan_line.ep().y()-map_line.ep().y())*(scan_line.ep().y()-map_line.ep().y()) );


            float dist2 = find_min_quadratic_eq(a2, b2, c2);

            if(!std::isnan(dist1) && !std::isnan(dist2)) {
                // float dist1 = a1*lambda1*lambda1 + b1*lambda1 + c1;
                // float dist2 = a2*lambda2*lambda2 + b2*lambda2 + c2;

                // std::cout << "dist: " << sqrt(dist1) << ", " << sqrt(dist2) << std::endl;
                // std::cout << "dist: " << dist1 << ", " << dist2 << std::endl;

                // float dist = (sqrt(dist1)+sqrt(dist2))/2;
                float dist = dist1+dist2;

                if(dist < min_dist) {
                    min_dist = dist;
                    matching_pair = std::make_pair(i, idx);
                }
            }
        }

        if( min_dist < (0.3)) {
            matching_pairs.insert(matching_pair);
            std::cout << matching_pair.first << ", " << matching_pair.second << ", " << min_dist << std::endl;
        }
    }

    return matching_pairs;
}

std::unordered_map<int, int> LineMatcher::find_matching_pair_from_simple_distance(std::vector<line_extractor::LineSegment>& scan_lines, 
                                                                                  std::vector<line_extractor::LineSegment>& map_lines, 
                                                                                  std::vector<int>& map_idx)
{
    std::unordered_map<int, int> matching_pairs;

    for(int i=0; i < scan_lines.size(); ++i) {

        float min_dist = std::numeric_limits<float>::max();
        std::pair<int, int> matching_pair;
        
        for(int idx:map_idx) {
            line_extractor::LineSegment& scan_line = scan_lines[i];
            line_extractor::LineSegment& map_line = map_lines[idx];

            if( fabs(line_extractor::LineSegment::calc_angle(scan_line, map_line)) > fabs(angle_threshold_))
                continue;

            float pdist_sp = map_line.dist_from_sp(scan_line.sp());
            float pdist_ep = map_line.dist_from_ep(scan_line.ep());

            float ldist_sp = map_line.dist_from_line(scan_line.sp());
            float ldist_ep = map_line.dist_from_line(scan_line.ep());

            if (ldist_sp > 0.25 || ldist_ep > 0.25)
                continue;

            float min_dist_sp = (pdist_sp < ldist_sp) ? pdist_sp : ldist_sp;
            float min_dist_ep = (pdist_ep < ldist_ep) ? pdist_ep : ldist_ep;

            float dist = (min_dist_sp + min_dist_ep)/2.0;

            if(dist < min_dist) {
                min_dist = dist;
                matching_pair = std::make_pair(i, idx);
            }
        }

        if( min_dist < (0.5)) {
            matching_pairs.insert(matching_pair);
            std::cout << matching_pair.first << ", " << matching_pair.second << ", " << min_dist << std::endl;
        }
    }

    return matching_pairs;
}

float LineMatcher::find_min_quadratic_eq(float a, float b, float c)
{
    float min_val = std::numeric_limits<float>::quiet_NaN();
    if (a > 0.0) {
        float lambda = -b / (2.0*a);

        if(lambda < 0.0)
            lambda = 0.0;
        else if(lambda > 1.0)
            lambda = 1.0;

        min_val = a*lambda*lambda + b*lambda + c;
        
        // min_val = c-b*b / (4.0*a);
    } //else {
        // std::cerr << a << "x^2 + " << b << "x + " << c << std::endl;
    //}
    if (min_val < 0.0)
        min_val = std::numeric_limits<float>::quiet_NaN();

    return min_val;
}

void LineMatcher::reset()
{

}

}