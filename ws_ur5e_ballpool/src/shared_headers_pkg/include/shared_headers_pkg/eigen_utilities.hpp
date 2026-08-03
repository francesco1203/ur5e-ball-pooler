#pragma once   // evita inclusioni doppie — alternativa moderna agli include guard

#include <Eigen/Dense>

//Alias
using Vector3d = Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using VectorXd = Eigen::VectorXd;                    //generic Vector dinamico
using MatrixXd = Eigen::MatrixXd;                    //generic Matrix dinamica
using Quaternion = Eigen::Quaterniond;
using RotationAxis = Eigen::AngleAxisd;              //rotazione attorno ad un asse (usato per definire quaternioni)
using RotoTraslMatrix = Eigen::Isometry3d;


//assi per rotazioni
const Vector3d X_AXIS= Eigen::Vector3d::UnitX();
const Vector3d Y_AXIS = Eigen::Vector3d::UnitY();
const Vector3d Z_AXIS = Eigen::Vector3d::UnitZ();

