#pragma once
#include <Eigen/Geometry>
#include <line_assembler/LineFactor.hpp>

struct Pose2d {
    float x;
    float y;
    float theta;

    Eigen::Isometry2f get_transform() {
        Eigen::Rotation2Df rot(theta);
        Eigen::Isometry2f transformation = Eigen::Isometry2f::Identity();
        transformation.translation() = Eigen::Vector2f(x, y);
        transformation.linear() = rot.toRotationMatrix();

        return transformation;
    }
};

inline std::istream& operator>>(std::istream& input, Pose2d& pose) {
    input >> pose.x >> pose.y >> pose.theta;

    pose.theta = NormalizeAngle(pose.theta);

    return input;
}