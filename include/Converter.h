/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>
#include"Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include"Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

/**
 * Converter between g2o and opencv.
 * Class not instantiated, without properties, set of class methods.
 * Converts elements between opencv and g2o expression forms.
 * Converts in both directions Point, Vector, Mat, Egien :: Matrix, g2o :: sim3, g2o :: SE2Quat.
 */

class Converter
{
public:
	/** Converts a Mat matrix of descriptors into a vector of Mat descriptors. */
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

	 /** Converts a Mat to SE3Quat. */
    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);

	/** Converts from Sim3 to SE3Quat. */
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

	 /** Converts SE3Quat to Mat. */
    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);

	/** Converts Sim3 to Mat. */
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);

	/** Converts Eigen matrices to Mat. */
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);

	 /** Converts Eigen matrices to Mat. */
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);

	/** Converts Eigen matrices to Mat. */
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);

	/** Converts Eigen matrices from Eigen to Mat. */
    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);

	 /** Converts a 3D vector from Mat to Eigen. */
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);

	 /** Converts Point3f to Eigen. */	
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);

	/** Converts Mat to Eigen. */
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);

	/** Convert Mat to vector. */
    static std::vector<float> toQuaternion(const cv::Mat &M);
};

}// namespace ORB_SLAM

#endif // CONVERTER_H
