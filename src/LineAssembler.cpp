#include <line_assembler/LineAssembler.h>
#include <line_assembler/LineFactor.hpp>

#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>

#include <tf/tf.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace line_assembler {

LineAssembler::LineAssembler(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom",1,&LineAssembler::odom_callback, this);
    lines_sub_ = nh.subscribe<line_extractor::LineListMessage>("/lines",1, &LineAssembler::lines_callback, this);

    scan_marker_pub_ = nh.advertise<visualization_msgs::Marker>("global_scan_line", 10);
    map_marker_pub_ = nh.advertise<visualization_msgs::Marker>("global_line", 10);

    map_ = std::make_shared<LineMap>();
    matcher_ = std::make_shared<LineMatcher>();
    reset();
}

void LineAssembler::reset()
{

}

void LineAssembler::odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    // std::cout << "odom callback" << std::endl;
    odom_msg->pose.pose.position.x;
    odom_msg->pose.pose.position.y;
    odom_msg->pose.pose.position.z;

    odom_msg->pose.pose.orientation.x;
    odom_msg->pose.pose.orientation.y;
    odom_msg->pose.pose.orientation.z;
    odom_msg->pose.pose.orientation.w;

    geometry_msgs::Pose2D pose2d;
    pose2d.x = odom_msg->pose.pose.position.x;
    pose2d.y = odom_msg->pose.pose.position.y;
    
    tf::Quaternion q(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    pose2d.theta = yaw;

    Pose2d cur_odom;
    cur_odom.x = static_cast<float>(pose2d.x);
    cur_odom.y = static_cast<float>(pose2d.y);
    cur_odom.theta = static_cast<float>(pose2d.theta);

    Eigen::Isometry2f tf_cur = cur_odom.get_transform();
    // std::cout << tf_cur.matrix() << std::endl;

    T_prev_cur_ = prev_odom_.get_transform().inverse() * cur_odom.get_transform();

    Eigen::Isometry2f T_w_c = pose_.get_transform() * T_prev_cur_;

    pose_.x = T_w_c.translation()(0);
    pose_.y = T_w_c.translation()(1);
    pose_.theta = Eigen::Rotation2Df(T_w_c.rotation()).angle();

    prev_odom_.x = cur_odom.x;
    prev_odom_.y = cur_odom.y;
    prev_odom_.theta = cur_odom.theta;

    // std::cout << T_w_c.translation() << std::endl;
    // std::cout << Eigen::Rotation2Df(T_w_c.rotation()).angle() << std::endl;

    // std::cout << x_ << ", " << y_ << ", " << theta_ << std::endl;
}

void LineAssembler::lines_callback(const line_extractor::LineListMessage::ConstPtr& lines_msg)
{
    std::cout << "lines callback at time " << lines_msg->header.stamp.toSec() << std::endl;
    line_segments_.clear();
    local_line_segments_.clear();
    std::unordered_map<int, int> matching_pairs;

    for(line_extractor::LineMessage line_msg:lines_msg->lines) {
        line_extractor::LineSegment line_seg(line_msg);

        local_line_segments_.push_back(line_seg);

        line_seg.param_update(pose_.x, pose_.y, pose_.theta);
        line_segments_.push_back(line_seg);
    }

    if(!map_->init()) {
        map_->add_lines(line_segments_);
    } else {
        matching_pairs = matcher_->time_based_matching(line_segments_, map_->lines());
        std::vector<line_extractor::LineSegment> new_map_candidate;

        ceres::LossFunction* loss_function = NULL;
        ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(params, 1);
        problem.AddParameterBlock(params+1, 1);
        problem.AddParameterBlock(params+2, 1, angle_local_parameterization);

        params[0] = static_cast<double> (pose_.x);
        params[1] = static_cast<double> (pose_.y);
        params[2] = static_cast<double> (pose_.theta);

        for(int i=0; i<line_segments_.size(); ++i) {
            if(matching_pairs.find(i) != matching_pairs.end()) {
                int map_idx = matching_pairs[i];

                ceres::CostFunction* cost_function = LineFactor::Create(local_line_segments_[i], map_->lines()[map_idx]);
                problem.AddResidualBlock(cost_function, loss_function, params, params+1, params+2);
            }
        }

        if (matching_pairs.size() > 0) {
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            // options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
            options.max_num_iterations = 40;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            // std::cout << summary.FullReport() << '\n';

            // pose_.x = params[0];
            // pose_.y = params[1];
            pose_.theta = params[2];

            prev_odom_.x = pose_.x;
            prev_odom_.y = pose_.y;
            prev_odom_.theta = pose_.theta;
        }

        for(int i=0; i<line_segments_.size(); ++i) {
            if(matching_pairs.find(i) != matching_pairs.end()) {
                int map_idx = matching_pairs[i];

                local_line_segments_[i].param_update(pose_.x, pose_.y, pose_.theta);
                map_->lines()[map_idx].merge_line(local_line_segments_[i]);
                // map_->lines()[map_idx].merge_line(line_segments_[i]);
                std::cout << "line merged" << std::endl;
            } else {
                if(line_segments_[i].num() > 4 && line_segments_[i].length() > 0.5) {
                    local_line_segments_[i].param_update(pose_.x, pose_.y, pose_.theta);
                    new_map_candidate.push_back(local_line_segments_[i]);
                    // new_map_candidate.push_back(line_segments_[i]);
                }
            }
        }

        // for(int i=0; i<line_segments_.size(); ++i) {
        //     if(matching_pairs.find(i) != matching_pairs.end()) {
        //         int map_idx = matching_pairs[i];

        //         map_->lines()[map_idx].merge_line(line_segments_[i]);
        //         std::cout << "line merged" << std::endl;
        //     } else {
        //         if(line_segments_[i].num() > 4 && line_segments_[i].length() > 0.5) {
        //             new_map_candidate.push_back(line_segments_[i]);
        //         }
        //     }
        // }


        map_->add_lines(new_map_candidate);

        //  std::vector<line_extractor::LineSegment> matched_lines;
        //  for(auto iter = matching_pairs.begin(); iter != matching_pairs.end(); ++iter) {
            //  matched_lines.push_back(line_segments_[iter->first]);
            // map_->add_line(line_segments_[iter->first]);
        //  }
        //  map_->add_lines(matched_lines);
    }

    // Publish visualization msg
    visualization_msgs::Marker scan_list;
    scan_list.header = lines_msg->header;
    scan_list.id = 2;
    scan_list.type = visualization_msgs::Marker::LINE_LIST;
    scan_list.scale.x = 0.1;
    scan_list.color.r = 1.0;
    scan_list.color.g = 1.0;
    scan_list.color.a = 1.0;

    visualization_msgs::Marker line_list;
    line_list.header = lines_msg->header;
    line_list.id = 3;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
    line_list.color.r = 1.0;
    line_list.color.g = 0.2;
    line_list.color.a = 1.0;

    for(auto iter = matching_pairs.begin(); iter != matching_pairs.end(); ++iter) {
        int scan_id = (*iter).first;
        int map_id = (*iter).second;

        line_extractor::LineSegment& scan_line = line_segments_[scan_id];
        line_extractor::LineSegment& map_line = map_->lines()[map_id];

        geometry_msgs::Point sp_msg, ep_msg;
        sp_msg.x = scan_line.sp().x();
        sp_msg.y = scan_line.sp().y();
        ep_msg.x = scan_line.ep().x();
        ep_msg.y = scan_line.ep().y();

        scan_list.points.push_back(sp_msg);
        scan_list.points.push_back(ep_msg);

        // sp_msg.x = map_line.sp().x();
        // sp_msg.y = map_line.sp().y();
        // ep_msg.x = map_line.ep().x();
        // ep_msg.y = map_line.ep().y();

        // line_list.points.push_back(sp_msg);
        // line_list.points.push_back(ep_msg);
    }

    scan_marker_pub_.publish(scan_list);

    // visualization_msgs::Marker line_list;
    // line_list.header = lines_msg->header;
    // line_list.id = 2;
    // line_list.type = visualization_msgs::Marker::LINE_LIST;
    // line_list.scale.x = 0.1;
    // line_list.color.r = 1.0;
    // line_list.color.a = 1.0;

    for(line_extractor::LineSegment line_segment:map_->lines())
    {
        // if(line_segment.num() < 4) continue;

        geometry_msgs::Point sp_msg, ep_msg;
        sp_msg.x = line_segment.sp().x();
        sp_msg.y = line_segment.sp().y();
        ep_msg.x = line_segment.ep().x();
        ep_msg.y = line_segment.ep().y();

        line_list.points.push_back(sp_msg);
        line_list.points.push_back(ep_msg);
    }

    map_marker_pub_.publish(line_list);
    // marker_pub_.publish(line_list);
}

}