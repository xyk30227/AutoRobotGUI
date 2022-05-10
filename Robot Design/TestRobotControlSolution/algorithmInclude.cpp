#include <algorithmZysInclude.h>
#include <hand_eye/hand_eye.h>
#include <RobotScan/RobotScan.h>
#include <thirdGuide/GuideHandEye.h>
#include "Sn3DCalibration/Sn3DCalibrationDefines.h"
#include "Sn3DCalibration/tools/CalibrationTableRecognition.h"
#include "Sn3DCalibration/tools/MarkPointRecognition.h"
#include "Sn3DCalibration/Sn3DCalibrationInclude.h"
#include "Sn3DCalibration/tools/commonTools.h"
#include "DataArchive/DataArchive.h"
#include "common/Sn3DLock.h"
#include <thread>

namespace Sn3DAlgorithm{

	RetVal handeye_Cali(const std::vector<ImageGreyb>& imageL, 
		const std::vector<ImageGreyb>& imageR, 
		const CameraParam& cameraL, 
		const CameraParam& cameraR,
		const RigidMatrix& matL, 
		const RigidMatrix& matR,
		const std::vector<Eigen::Vector3d>& robotPos,
		Eigen::Matrix4d& T, double& error)
	{
		if (!sn3DLockInstance.is_has_dongle()) return RetVal_NO_DONGLE;
		//debug
		debug_archive_data("handeye_Cali", imageL, "imageL", imageR, "imageR", robotPos, "RoPos");
		HandEye hand;
		hand.init_handeye(cameraL,cameraR,matL,matR,robotPos,imageL,imageR);
		auto val = hand.run_hand_eye(T,error);
		if (val != RetVal_OK) return val;
		return RetVal_OK;
	}

	RetVal hand_eye_calibration_pts(
		const std::vector<Eigen::Vector3d>& robotPts,
		const std::vector<Eigen::Vector3d>& scannerPts,
		RigidMatrix& rt,
		double& error)
	{
		if (!sn3DLockInstance.is_has_dongle()) return RetVal_NO_DONGLE;
		debug_archive_data("hand_eye_calibration_pts", robotPts, "robotPts", scannerPts, "scannerPts");

		///< 输入检测
		if (robotPts.size() != scannerPts.size() || robotPts.size() != 4)
		{
			std::cout << "point pair less than four group！" << std::endl;
			error = -1;
			return RetVal_ILLEGAL_INPUT;
		}

		///< 判断输入点对是否共线
		HandEye h;
		auto val = h.is_point_valid(robotPts);
		if (val != RetVal_OK) {
			std::cout << "is_point_valid failed!" << std::endl;
			error = -1;
			return val;
		}

		///< 求解手眼关系矩阵
		auto pose_Estimation = [](const std::vector<Eigen::Vector3d>& pts1, const std::vector<Eigen::Vector3d>& pts2, Eigen::Matrix<double, 3, 3>& R, Eigen::Matrix<double, 3, 1>& t)
		{
			if (pts1.size() != 4 || pts2.size() != 4)
			{
				Sn_Error("Illegal input");
				return RetVal_ILLEGAL_INPUT;
			}

			int num = 0;
			num = pts1.size();
			Eigen::Vector3d p1, p2;//center of point mass
			p1.setZero();
			p2.setZero();
			for (int i = 0; i < num; i++)
			{
				p1 += pts1[i];
				p2 += pts2[i];
			}

			p1 = p1 / num;
			p2 = p2 / num;

			std::vector<Eigen::Vector3d> q1(num), q2(num);
			for (int i = 0; i < num; i++)
			{
				q1[i] = pts1[i] - p1;
				q2[i] = pts2[i] - p2;
			}


			Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
			for (int i = 0; i < num; i++)
			{
				W += Eigen::Vector3d(q2[i][0], q2[i][1], q2[i][2])*Eigen::Vector3d(q1[i][0], q1[i][1], q1[i][2]).transpose();
			}
			//SVD on W
			Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
			Eigen::Matrix3d U = svd.matrixU();
			Eigen::Matrix3d V = svd.matrixV();
			R = V*(U.transpose());
			if (R.determinant() < 0)
			{
				V(0, 2) = -1 * V(0, 2);
				V(1, 2) = -1 * V(1, 2);
				V(2, 2) = -1 * V(2, 2);
				R = V*(U.transpose());
			}
			t = Eigen::Vector3d(p1[0], p1[1], p1[2]) - R*Eigen::Vector3d(p2[0], p2[1], p2[2]);
			return RetVal_OK;
		};
		Eigen::Matrix3d R;Eigen::Vector3d t;
		val = pose_Estimation(robotPts, scannerPts, R, t);
		if (val != RetVal_OK) {
			std::cout << "pose_Estimation failed!" << std::endl;
			error = -1;
			return val;
		}

		///< 计算误差
		error = 0.0; double errorX = 0.0; double errorY = 0.0; double errorZ = 0.0;
		for (int i = 0; i < scannerPts.size(); ++i){
			const auto&  errorPoint = (R*scannerPts[i] + t) - robotPts[i];
			errorX += sqrt(errorPoint[0] * errorPoint[0]);
			errorY += sqrt(errorPoint[1] * errorPoint[1]);
			errorZ += sqrt(errorPoint[2] * errorPoint[2]);
			error += ((R*scannerPts[i] + t) - robotPts[i]).norm();                                                                                                             
		}
		error = error / static_cast<double>(scannerPts.size());
		std::cout << "error XYZ: " << errorX / static_cast<double>(scannerPts.size()) << "," 
			<< errorY / static_cast<double>(scannerPts.size()) << "," << errorZ / static_cast<double>(scannerPts.size()) << std::endl;

		///< 更新rt
		rt.set_identity();
		rt.set_rotation(R);
		rt.set_translation(t);

		return RetVal_OK;
	}

	RetVal SN3D_HANDEYE_API auto_hand_eye_calibration(
		const std::vector<Eigen::Vector4d>& robotPts,
		const std::vector<Eigen::Vector3d>& scannerPts,
		Eigen::Matrix4d& T,
		double& error)
	{
		if (!sn3DLockInstance.is_has_dongle()) return RetVal_NO_DONGLE;
		debug_archive_data("hand_eye_calibration_pts", robotPts, "robotPts", scannerPts, "scannerPts");

		///< 输入检测
		if (robotPts.size() != scannerPts.size() || robotPts.size() < 6)
		{
			std::cout << "point pair less than six group！" << std::endl;
			error = -1;
			return RetVal_ILLEGAL_INPUT;
		}

		///< 机器人位姿转换
		std::vector<RigidMatrix> robot(robotPts.size(),RigidMatrix());
		for (int i = 0; i < robotPts.size();++i)
		{
			const auto& tran = robotPts[i].block<3, 1>(0, 0);
			const auto& theta = robotPts[i][3]/180*PI;
			
			Eigen::AngleAxisd rotZ(theta,Eigen::Vector3d::UnitZ());///绕Z轴旋转
			Eigen::Matrix3d rot = rotZ.matrix();

			robot[i].set_translation(tran);
			robot[i].set_rotation(rot);
		}

		///< 执行多点法自动手眼标定
		RigidMatrix mat; GuiderHandEye gh;
		auto val = gh.auto_hand_eye(scannerPts, robot, mat, error);
		if (val != RetVal_OK){
			std::cout << "auto_hand_eye failed!" << std::endl;
			error = -1;
			return val;
		}

		///< 输出矩阵类型转换
		T.setIdentity();
		T.block<3, 3>(0, 0) = mat.get_rotation();
		T.block<3, 1>(0, 3) = mat.get_translation();

		return RetVal_OK;
	}

	RetVal show_edge(
		const ImageGreyb& imageL, 
		const ImageGreyb& imageR, 
		std::vector<EllipseResult>& ellipse_L, 
		std::vector<EllipseResult>& ellipse_R)
	{
		if (!sn3DLockInstance.is_has_dongle()) return RetVal_NO_DONGLE;
		//debug
		debug_archive_data("showEdge", imageL, "imageL", imageR, "imageR");
		HandEye handeye;
		RetVal val = handeye.show_edge(imageL, imageR, ellipse_L, ellipse_R);
		if (val != RetVal_OK) return val;
		return RetVal_OK;
	}

	RetVal robot_guide_calibration(
		const std::vector<ImageGreyb>& imageL, 
		const std::vector<ImageGreyb>& imageR, 
		const CameraParam& cameraL, 
		const CameraParam& cameraR,
		const RigidMatrix& matRL, 
		const std::vector<RigidMatrix>& robotPose, 
		const HandEyeWay&hway, 
		RigidMatrix& handEyeTrans, 
		double& error)
	{
		if (!sn3DLockInstance.is_has_dongle()) return RetVal_NO_DONGLE;
		//debug
		debug_archive_data("robot_guide_calibration", imageL, "imageL", imageR, "imageR", cameraL, "cameraL", cameraR, "cameraR", matRL, "matRL", robotPose, "robotPose");
		GuiderHandEye gh;
		ScanHandEye sh;
		
		//1、设置左右相机参数
		gh.set_CameraParam(cameraL, cameraR, matRL);
		std::vector<std::vector<Eigen::Vector3d>> pts;

		//2、重建左右图像的3d点
		if (imageL.size()!=imageR.size())
		{
			std::cout << "left image num is not equal to right!" << std::endl;
			return RetVal_ILLEGAL_INPUT;
		}
		std::vector<ImageGreyf> imageLf(imageL.size()), imageRf(imageR.size());
		for (int i = 0; i < imageL.size();i++)
		{
			image_conversion(imageL[i], imageLf[i]);
			image_conversion(imageR[i], imageRf[i]);
		}

		RetVal val = gh.reconstuct_point_left(imageLf, imageRf, pts);
		if (val!=RetVal_OK)
		{
			std::cout << "ReconstuctPoint 3d point failed!" << std::endl;
			return RetVal_RUNNING_ERROR;
		}

		// 机器人末端位姿转换
		std::vector<RigidMatrix> robotT;
		if (sh.robot_transform(robotPose, robotT, hway) != RetVal_OK)
			return RetVal_RUNNING_ERROR;

		// 相机相对于标定板的位姿变化
		std::vector<RigidMatrix> cameraT;
		std::vector<bool> valid;
		if ((gh.calibTable_estimation(pts, cameraT, valid)) != RetVal_OK)
			return RetVal_RUNNING_ERROR;

		std::vector<RigidMatrix> vrobotT, vcameraT;
		vrobotT.clear();
		vcameraT.clear();
		for (int i = 0; i < valid.size();++i)
		{
			if (!valid[i]) continue;
			vrobotT.push_back(robotT[i]);
			vcameraT.push_back(cameraT[i]);
		}

		//3、求解AX=XB方程，得出X的初值
		if (sh.estimate_HandEye(vrobotT, vcameraT, handEyeTrans) != RetVal_OK)
			return RetVal_RUNNING_ERROR;
		std::cout << "init hand-eye matrix:  " << std::endl;
		std::cout << handEyeTrans.get_rotation() << std::endl;
		std::cout << handEyeTrans.get_translation() << std::endl;
		
		//4、根据A'XP或AXP恒定进行非线性优化
		std::vector<RigidMatrix> invRobotT(robotPose.size());
		for (int i = 0; i < robotPose.size(); i++)
		{
			if (hway == Eye_To_Hand)
				invRobotT[i] = robotPose[i].inverse();
			else if (hway == Eye_In_Hand)
				invRobotT[i] = robotPose[i];
		}
		long t1 = clock();
		val = sh.handEye_calibration(invRobotT, pts, handEyeTrans, error);
		long t2 = clock();
		std::cout << "optimize time is " << t2 - t1 << std::endl;

		if (val != RetVal_OK)
		{
			std::cout << "handEye_calibration RT failed!" << std::endl;
			return RetVal_RUNNING_ERROR;
		}
		return RetVal_OK;
	}

	RetVal judge_robot_calib(
		const ImageGreyb& imageL, 
		const ImageGreyb& imageR, 
		const CameraParam& cameraL, 
		const CameraParam& cameraR,
		const RigidMatrix& matRL, 
		int& numPointL, 
		int& numPointR, 
		Eigen::Vector3d& centerCalibTable)
	{
		if (!sn3DLockInstance.is_has_dongle()) return RetVal_NO_DONGLE;
		//debug
		debug_archive_data("judge_robot_calib", imageL, "imageL", imageR, "imageR", cameraL, "cameraL", cameraR, "cameraR", matRL, "matRL");
		CameraCommonCaliParam cameraParamL, cameraParamR;
		cameraParamL.K = cameraL.K;
		cameraParamL.dis = cameraL.dis;
		cameraParamR.K = cameraR.K;
		cameraParamR.dis = cameraR.dis;

		RobotCalibRecognition robotCalib;
		GuassFilterParam guassParam;
		guassParam.iFilterH = 5;
		guassParam.iFilterW = 5;
		guassParam.guasssigma = 1.0;
		CannyParam cannyParam;
		cannyParam.thresholdHigh = 0.25;
		cannyParam.thresholdLow = 0.15;
		cannyParam.sigma = 1.0;
		CvCannyParam cvcannyparam;
		cvcannyparam.thresholdHigh = 100;
		cvcannyparam.thresholdLow = 50;
		EllipseParam eParam;
		eParam.pointQuality = 0.3;
		eParam.maxDiameter = 100;
		eParam.minDiameter = 2.0;
		eParam.circularity = 2.0;

		robotCalib.set_guass_filter_param(guassParam);
		robotCalib.set_canny_param(cannyParam);
		robotCalib.set_ellipse_param(eParam);
		robotCalib.set_cvCanny_param(cvcannyparam);

		std::vector<Eigen::Vector2d> markPointsPosL, markPointsPosR;
		ImageGreyf imageLf, imageRf;
		image_conversion(imageL, imageLf);
		image_conversion(imageR, imageRf);
		if (robotCalib.recognition_robotcalib(imageLf, markPointsPosL) != RetVal_OK)
		{
			numPointL = -1;
			centerCalibTable[0] = -1;
			centerCalibTable[1] = -1;
			centerCalibTable[2] = -1;
			return RetVal_RUNNING_ERROR;
		}
		numPointL = markPointsPosL.size();
		if (robotCalib.recognition_robotcalib(imageRf, markPointsPosR) != RetVal_OK)
		{
			numPointR = -1;
			centerCalibTable[0] = -1;
			centerCalibTable[1] = -1;
			centerCalibTable[2] = -1;
			return RetVal_RUNNING_ERROR;
		}
		numPointR = markPointsPosR.size();

		std::vector<Eigen::Vector3d> outPut;
		GuiderHandEye gh;
		if (numPointL != numPointR || numPointL != 4)
		{
			return RetVal_RUNNING_ERROR;
			centerCalibTable[0] = -1;
			centerCalibTable[1] = -1;
			centerCalibTable[2] = -1;
		}

		gh.mark_point_reconstruction_mid(cameraParamL, cameraParamR, matRL, markPointsPosL, markPointsPosR, outPut);
		centerCalibTable = (outPut[0] + outPut[2] + outPut[3]) / 3;
		//xy坐标取左图像坐标
		centerCalibTable[0] = (markPointsPosL[0].x() + markPointsPosL[2].x() + markPointsPosL[3].x()) / 3;
		centerCalibTable[1] = (markPointsPosL[0].y() + markPointsPosL[2].y() + markPointsPosL[3].y()) / 3;
		return RetVal_OK;
	}

	RetVal robot_scan_calibration(
		const std::vector<Eigen::Matrix<double, 3, 2>>& robotPose, 
		const std::vector<std::vector<Eigen::Vector3d>>& p3D, 
		RigidMatrix& handEyeTrans)
	{
		if (!sn3DLockInstance.is_has_dongle()) return RetVal_NO_DONGLE;
		//debug
		debug_archive_data("robot_scan_calibration", robotPose, "robotPose", p3D, "p3D");
		ScanHandEye scanRobot;

		//机器人末端位姿转换x,y,z rx,ry,rz转为刚体矩阵
		std::vector<RigidMatrix> robotT;
		double error = 0;
		if (scanRobot.roPos_transform(robotPose, robotT) == RetVal_OK)
		{
			std::vector<RigidMatrix> invRobotT(robotT.size());
			for (int i = 0; i < robotT.size(); i++)
			{
				invRobotT[i] = robotT[i].inverse();
			}
			scanRobot.handEye_calibration(robotT, p3D, handEyeTrans,error);
		}
		else
		{
			return RetVal_ILLEGAL_INPUT;
		}
		return RetVal_OK;
	}

	RetVal judge_calibTable(
		const ImageGreyb& imageL,
		const ImageGreyb& imageR, 
		int& numPointL, 
		int& numPointR, 
		Eigen::Vector2d& centerCalibTable)
	{
		if (!sn3DLockInstance.is_has_dongle()) return RetVal_NO_DONGLE;
		//debug
		debug_archive_data("judge_calibTable", imageL, "imageL", imageR, "imageR");
		//标定板识别
		CalibrationTableRecognition rec;
		GuassFilterParam guassParam;
		guassParam.iFilterH = 5;
		guassParam.iFilterW = 5;
		guassParam.guasssigma = 1.0;
		CannyParam cannyParam;
		cannyParam.thresholdHigh = 0.25;
		cannyParam.thresholdLow = 0.15;
		cannyParam.sigma = 1.0;
		CvCannyParam cvcannyparam;
		cvcannyparam.thresholdHigh = 160;
		cvcannyparam.thresholdLow = 80;
		EllipseParam eParam;
		eParam.pointQuality = 0.2;
		eParam.maxDiameter = 150.0;
		eParam.minDiameter = 4.0;
		eParam.circularity = 1.5;
		CalibrationTableParam calTableParam;//设置先临自己的标定板参数
		calTableParam.inOutColor = static_cast<InOutColor>(1);
		calTableParam.bigCircleNum = 4;
		calTableParam.radiusRatio = 1.5;
		calTableParam.smallNumThreshold = 100;
		calTableParam.num_X = 21;
		calTableParam.num_Y = 21;
		calTableParam.gap_X = 25;
		calTableParam.gap_Y = 25;
		calTableParam.center_X = 8;
		calTableParam.center_Y = 6;

		rec.set_calibTable_param(calTableParam);
		rec.set_guass_filter_param(guassParam);
		rec.set_canny_param(cannyParam);
		rec.set_ellipse_param(eParam);
		rec.set_cvCanny_param(cvcannyparam);

		std::vector<Eigen::Vector2d> markPointsPosL, markPointsPosR;
		std::vector<Eigen::Vector2d> markPointsPosLL, markPointsPosRR;
		ImageGreyf imageLf, imageRf;
		

		image_conversion(imageR, imageRf);
		if (rec.recognition_shining3d_calibTable(imageRf, markPointsPosR) != RetVal_OK)
		{
			numPointR = -1;
			centerCalibTable[0] = -1;
			centerCalibTable[1] = -1;
			return RetVal_RUNNING_ERROR;
		}
		for (int i = 0; i < markPointsPosR.size(); ++i)
		{
			if (markPointsPosR[i][0] < 0 || markPointsPosR[i][1] < 0)
			{
				continue;
			}
			markPointsPosRR.push_back(markPointsPosR[i]);
		}
		numPointR = markPointsPosRR.size();

		image_conversion(imageL, imageLf);
		if (rec.recognition_shining3d_calibTable(imageLf, markPointsPosL) != RetVal_OK)
		{
			numPointL = -1;
			centerCalibTable[0] = -1;
			centerCalibTable[1] = -1;
			return RetVal_RUNNING_ERROR;
		}
		for (int i = 0; i < markPointsPosL.size(); ++i)
		{
			if (markPointsPosL[i][0] < 0 || markPointsPosL[i][1] < 0)
			{
				continue;
			}
			markPointsPosLL.push_back(markPointsPosL[i]);
		}
		numPointL = markPointsPosLL.size();
		centerCalibTable = 0.5*(markPointsPosL[225]+markPointsPosL[215]);

		return RetVal_OK;
	}

	RetVal run_scanner_calibration(
		const std::vector<ImageGreyb>& imageL, 
		const std::vector<ImageGreyb>& imageR,
		const int& imgWidth,const int& imgHeight,
		const std::vector<DistanceRef>& disRefs,
		const std::vector<std::string>& leftCCF, 
		const std::vector<std::string>& rightCCF, 
		double& error, bool encry)

	{
		if (!sn3DLockInstance.is_has_dongle()) return RetVal_NO_DONGLE;
		//debug
		debug_archive_data("run_scanner_calibration", imageL, "imageL", imageR, "imageR", leftCCF, "leftCCF", rightCCF, "rightCCF", encry, "encry");
		//输入检测
		if (imageL.size() != imageR.size() || imageL.size() < 3)
		{
			std::cout << "illegal input for image!" << std::endl;
			return RetVal_ILLEGAL_INPUT;
		}
		if (leftCCF.size() < 1 || rightCCF.size() < 1 || leftCCF.size() != rightCCF.size())
		{
			std::cout << "illegal input for path!" << std::endl;
			return RetVal_ILLEGAL_INPUT;
		}
		//标定板识别
		CalibrationTableRecognition rec;
		GuassFilterParam guassParam;
		guassParam.iFilterH = 5;
		guassParam.iFilterW = 5;
		guassParam.guasssigma = 1.0;
		CannyParam cannyParam;
		cannyParam.thresholdHigh = 0.25;
		cannyParam.thresholdLow = 0.15;
		cannyParam.sigma = 1.0;
		CvCannyParam cvcannyparam;
		cvcannyparam.thresholdHigh = 100;
		cvcannyparam.thresholdLow = 50;
		EllipseParam eParam;
		eParam.pointQuality = 0.3;
		eParam.maxDiameter = 150.0;
		eParam.minDiameter = 4.0;
		eParam.circularity = 1.5;
		CalibrationTableParam calTableParam;//设置先临自己的标定板参数
		calTableParam.inOutColor = static_cast<InOutColor>(1);
		calTableParam.bigCircleNum = 4;
		calTableParam.radiusRatio = 1.5;
		calTableParam.smallNumThreshold = 100;
		calTableParam.num_X = 21;
		calTableParam.num_Y = 21;
		calTableParam.gap_X = 25;
		calTableParam.gap_Y = 25;
		calTableParam.center_X = 8;
		calTableParam.center_Y = 6;

		rec.set_calibTable_param(calTableParam);
		rec.set_guass_filter_param(guassParam);
		rec.set_canny_param(cannyParam);
		rec.set_ellipse_param(eParam);
		rec.set_cvCanny_param(cvcannyparam);

		std::vector<Eigen::Vector2d> markPointsPosL, markPointsPosR;
		ImageGreyf imageLf, imageRf;
		std::vector<std::vector<Eigen::Vector2d>> p2DL(imageL.size(), std::vector<Eigen::Vector2d>(441)), p2DR(imageR.size(), std::vector<Eigen::Vector2d>(441));
		std::cout << "start CalibTable recognition!" << std::endl;
		for (int i = 0; i < imageL.size(); ++i)
		{
			//左图像
			image_conversion(imageL[i], imageLf);
			RetVal val = rec.recognition_shining3d_calibTable(imageLf, markPointsPosL);
			if (val != RetVal_OK)
			{
				std::cout << "recognition calibTable failed!" << std::endl;
				error = DBL_MAX;
				return RetVal_RUNNING_ERROR;
			}
			p2DL[i] = markPointsPosL;

			//右图像
			image_conversion(imageR[i], imageRf);
			val = rec.recognition_shining3d_calibTable(imageRf, markPointsPosR);
			if (val != RetVal_OK)
			{
				std::cout << "recognition calibTable failed!" << std::endl;
				error = DBL_MAX;
				return RetVal_RUNNING_ERROR;
			}
			p2DR[i] = markPointsPosR;

		}
		std::cout << "finish CalibTable recognition!" << std::endl;
		std::cout << std::endl;
		std::cout << "start camera calibration!" << std::endl;

		//双目标定初始化参数
		std::vector<double> KL = { 2000, imgWidth / 2.0, imgHeight / 2.0, 1, 0 };
		std::vector<double> KR = { 2000, imgWidth / 2.0, imgHeight / 2.0, 1, 0 };

		std::vector<double> disL(5, 0);
		std::vector<double> disR(5, 0);

		//构造3d点
		Eigen::Vector3d Init(-130, -156, 0);
		Eigen::Vector3d temp(0, 0, 0);
		std::vector<Eigen::Vector3d> p3D(441);
		int k = 0;
		for (int i = 0; i < 21; i++)
		{
			for (int j = 0; j < 21; j++)
			{
				temp[0] = Init[0] + j * 13;
				temp[1] = Init[1] + i * 13;
				p3D[k] = temp;
				k++;
				temp.setZero();
			}
		}

		//双目标定
		std::vector<std::vector<double>> quarL, tranL;
		Eigen::Vector3d QuarL, QuarR;//第一张图片相对于左右相机的旋转矢量
		Eigen::Vector3d TranL, TranR;//第一张图片相对于左右相机的平移矢量
		Eigen::Matrix3d rotL, rotR;//第一张图片相对于左右相机的旋转矩阵
		Eigen::Vector3d QuarLR, TranLR;//左相机到右相机的旋转矢量，平移矩阵
		Eigen::Matrix3d rotLR;//左相机到右相机的旋转矩阵

		std::vector<double> quarLR, tranLR;
		RetVal rey = Sn3D_double_camera_calibration(KL, disL, p2DL, KR, disR, p2DR, p3D, quarL, tranL, quarLR, tranLR, disRefs, error, true, true);
		if (rey != RetVal_OK)
		{
			std::cout << "double camera calibration failed!" << std::endl;
			return RetVal_RUNNING_ERROR;
		}
		else
		{
			std::cout << "camera calibration succeed!" << std::endl;
			//左相机
			QuarL[0] = quarL[0][0];
			QuarL[1] = quarL[0][1];
			QuarL[2] = quarL[0][2];
			TranL[0] = tranL[0][0];
			TranL[1] = tranL[0][1];
			TranL[2] = tranL[0][2];
			rodrigures(QuarL, rotL);

			//左右相机关系
			QuarLR[0] = quarLR[0];
			QuarLR[1] = quarLR[1];
			QuarLR[2] = quarLR[2];
			TranLR[0] = tranLR[0];
			TranLR[1] = tranLR[1];
			TranLR[2] = tranLR[2];
			rodrigures(QuarLR, rotLR);
			//右相机
			rotR = rotLR*rotL;
			TranR = TranLR + rotLR*TranL;
			rodrigures(rotR, QuarR);

			std::vector<double> quarll, tranll;
			std::vector<double> quarrr, tranrr;

			quarll.push_back(QuarL[0]);
			quarll.push_back(QuarL[1]);
			quarll.push_back(QuarL[2]);
			quarrr.push_back(QuarR[0]);
			quarrr.push_back(QuarR[1]);
			quarrr.push_back(QuarR[2]);

			tranll.push_back(TranL[0]);
			tranll.push_back(TranL[1]);
			tranll.push_back(TranL[2]);
			tranrr.push_back(TranR[0]);
			tranrr.push_back(TranR[1]);
			tranrr.push_back(TranR[2]);
			//标准文件输出
			double errorTmep = 0.0;

			//加密文件输出CCF:const char* leftCCF, const char* rightCCF
			double fcL[2] = { KL[0], KL[0] * KL[3] };
			double ccL[2] = { KL[1], KL[2] };
			double alphaL = KL[4];
			double dcL[5] = { disL[0], disL[1], disL[2], disL[3], disL[4] };
			double tranLN[3] = { tranll[0], tranll[1], tranll[2] };
			double quarLN[9] = { rotL(0, 0), rotL(0, 1), rotL(0, 2), rotL(1, 0), rotL(1, 1), rotL(1, 2), rotL(2, 0), rotL(2, 1), rotL(2, 2) };


			double fcR[2] = { KR[0], KR[0] * KR[3] };
			double ccR[2] = { KR[1], KR[2] };
			double alphaR = KR[4];
			double dcR[5] = { disR[0], disR[1], disR[2], disR[3], disR[4] };
			double tranRN[3] = { tranrr[0], tranrr[1], tranrr[2] };
			double quarRN[9] = { rotR(0, 0), rotR(0, 1), rotR(0, 2), rotR(1, 0), rotR(1, 1), rotR(1, 2), rotR(2, 0), rotR(2, 1), rotR(2, 2) };


			double fcError[2] = { 0.0, 0.0 };
			double ccError[2] = { 0.0, 0.0 };
			double alError = 0.0;
			double kcError[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
			double errorN = 0.0;
			if (encry)
			{
				GenerateOffset();
				for (int i = 0; i < leftCCF.size(); ++i)
				{
					WriteCcf_bin(leftCCF[i].c_str(), fcL, ccL, alphaL, dcL, fcError, ccError, alError, kcError, tranLN, quarLN, errorN, true);
					WriteCcf_bin(rightCCF[i].c_str(), fcR, ccR, alphaR, dcR, fcError, ccError, alError, kcError, tranRN, quarRN, errorN, false);
				}
			}
			else
			{
				for (int i = 0; i < leftCCF.size(); ++i)
				{
					write_ccf(leftCCF[i], KL, disL, quarll, tranll, errorTmep);
					write_ccf(rightCCF[i], KR, disR, quarrr, tranrr, errorTmep);
				}
			}
			//计算交点误差
#if 0
			CameraCommonCaliParam cameraL, cameraR;
			cameraL.K = KL; cameraL.dis = disL;
			cameraR.K = KR; cameraR.dis = disR;
			RigidMatrix matRL;
			matRL.set_rotation(rotLR);
			matRL.set_translation(TranLR);
			matRL.inverseSelf();
			std::vector<Eigen::Vector3d> p3dd;
			for (int i = 0; i < p2DL.size(); i++)
			{
				mark_point_reconstruction(cameraL, cameraR, matRL, p2DL[i], p2DR[i], p3dd);
			}
#endif	
			std::cout << "double camera calibration finish!" << std::endl;
		}
		return RetVal_OK;
	}
		

}