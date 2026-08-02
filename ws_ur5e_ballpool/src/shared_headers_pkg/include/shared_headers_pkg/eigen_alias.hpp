#pragma once   // evita inclusioni doppie — alternativa moderna agli include guard

#include <Eigen/Dense>

//Eigen
using Vector3d = Eigen::Vector3d;
using Quaternion = Eigen::Quaterniond;
using RotationAxis = Eigen::AngleAxisd;
//using RotoTraslMatrix = Eigen::Isometry3d;

//assi per rotazioni
const Vector3d X_AXIS= Eigen::Vector3d::UnitX();
const Vector3d Y_AXIS = Eigen::Vector3d::UnitY();
const Vector3d Z_AXIS = Eigen::Vector3d::UnitZ();