/*************************************************
* \brief 该文件定义四点法手眼标定
* \author 张远松
**************************************************/
#ifndef SN3D_ALGORITHM_SN3D_HANDEYE_CALIBRATION_H
#define SN3D_ALGORITHM_SN3D_HANDEYE_CALIBRATION_H
#include <stdio.h>
#include <iostream>
#include <tchar.h>
#include <math.h>
#include "Sn3DCalibration/Sn3DCalibrationDefines.h"
#include "Sn3DCalibration/tools/commonTools.h"
#include "Sn3DCalibration/tools/MarkPointRecognition.h"
#include "Sn3DCalibration/tools/ImagefilteringFunction.h"
#include "Sn3DCalibration/camera_calibration/sn3DCameraUtils.h"
#include "../algorithmZysInclude.h"
#include "../algorithmZysDefines.h"


namespace Sn3DAlgorithm
{
	class SN3D_HANDEYE_CLASS HandEye
	{
	public:
		HandEye();
		~HandEye();

	public:
		RetVal run_hand_eye(Eigen::Matrix4d& T, double& error);

		//初始化四点法手眼标定所需参数
		void init_handeye(
			const CameraParam& cameraL, 
			const CameraParam& cameraR, 
			const RigidMatrix& matL, 
			const RigidMatrix& matR,
			const std::vector<Eigen::Vector3d>& robotPos,
			const std::vector<ImageGreyb>& imageL,
			const std::vector<ImageGreyb>& imageR);
		
		//检测手眼关系标定板位置
		RetVal show_edge(
			const ImageGreyb& imageL, 
			const ImageGreyb& imageR, 
			std::vector<EllipseResult>& ellipse_L, 
			std::vector<EllipseResult>& ellipse_R);
		
		/*compute the hand-eye calibration error*/
		/*
		param [in]	    R:the rotation matrix betwwen the robot and scanner
		param [in]		t:the translation matrix betwwen the robot and scanner
		param [in]		p1:the mark point(3d point:world to the left camera)
		param [in]		p2:the robot point(TCP)
		param [out]     error:the hand-eye calibration error
		*/
		RetVal handeye_error(
			const Eigen::Matrix3d& R, 
			const Eigen::Vector3d& t, 
			const Eigen::Vector3d& p1, 
			const Eigen::Vector3d& p2, 
			double& error);

		/*judge the mark point is on the same line or not*/
		//param [in]        P: the mark point(3d point:world to the left camera)
		RetVal is_point_valid(const std::vector <Eigen::Vector3d>& P);

	private:
		/*Extract the mark point in the left and right image*/
		/*
		param [in]		imageRf:the right image
		param [in]		imageRf:the right image
		param [out]		pL:the mark point(2d) in the left image
		param [out]		pR:the mark point(2d) in the right image
		*/
		RetVal extract_2dpoint(
			const std::vector<ImageGreyb>& imageLb,
			const std::vector<ImageGreyb>& imageRb, 
			std::vector <Eigen::Vector2d>& pL,
			std::vector <Eigen::Vector2d>& pR);
		/*mark point reconstruction using the intersection of left polar line and right polar line*/
		/*param [in]	cameraL:the left camera param
		param [in]		cameraR:the right camera param
		param [in]		matRL:the matrix from the right camera to the left camera
		param [in]		p2DL:the left image mark point
		param [in]		p2DR:the right image mark point
		param [out]     output:the reconstruction 3d point relative to the left camera
		*/
		RetVal markpoint_reconstruct(
			const CameraCommonCaliParam& cameraL, 
			const CameraCommonCaliParam& cameraR,
			const RigidMatrix& matRL,
			const std::vector<Eigen::Vector2d>& p2DL, 
			const std::vector<Eigen::Vector2d>& p2DR, 
			std::vector<Eigen::Vector3d>& output);

		/*hand-eye calibration:3d pose estimation using SVD*/
		/*param [in]	pts1:the robot point(TCP)
		param [in]		pts2:the mark point(3d point:world to the left camera)
		param [out]		R:the rotation matrix betwwen the robot and scanner
		param [out]		t:the translation matrix betwwen the robot and scanner
		*/
		RetVal pose_Estimation_3d3d(
			const std::vector<Eigen::Vector3d>& pts1, 
			const std::vector<Eigen::Vector3d>& pts2, 
			Eigen::Matrix<double, 3, 3>& R,
			Eigen::Matrix<double, 3, 1>& t);

		CameraCommonCaliParam _cameraL, _cameraR;
		RigidMatrix _matL, _matR, _matRL;
		std::vector<Eigen::Vector3d> _RosP;
		std::vector<ImageGreyb> _imageLb, _imageRb;
		HandEyeImageParam _imageParam;
		Eigen::Vector3d _RobotP1, _ScannerP1;
		Eigen::Vector3d _RobotP2, _ScannerP2;
		Eigen::Vector3d _RobotP3, _ScannerP3;
	};

	void MatrixToEuler(Eigen::Matrix3d &R, Eigen::Vector3d& Euler);

	void eulerToMatrix(Eigen::Vector3d& Euler, Eigen::Matrix3d &R);
}


#endif