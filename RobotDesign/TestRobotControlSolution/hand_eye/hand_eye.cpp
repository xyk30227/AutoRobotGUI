#include "hand_eye/hand_eye.h"
#include <iomanip>
#include <thread>
using namespace Eigen;
namespace Sn3DAlgorithm
{
	HandEye::HandEye()
	{
		_imageParam.cannyParamL = 0.15;
		_imageParam.cannyParamH = 0.3;
		_imageParam.cvCannyParamL = 75;
		_imageParam.cvCannyParamH = 200;
		_imageParam.ellipseH = 200;
		_imageParam.ellipseL = 50;
		_imageParam.gaussParam = 3;
	}

	HandEye::~HandEye()
	{

	}

	//通过旋转矩阵计算对应的欧拉角
	void MatrixToEuler(Eigen::Matrix3d &R, Eigen::Vector3d& Euler)
	{
		//assert(isRotationMatrix(R));

		double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));

		bool singular = sy < 1e-6; // If

		double x, y, z;
		if (!singular) {
			x = atan2(R(2,1), R(2,2));
			y = atan2(-R(2,0), sy);
			z = atan2(R(1,0), R(0,0));
		}
		else {
			x = atan2(-R(1,2), R(1,1));
			y = atan2(-R(2,0), sy);
			z = 0;
		}
		Euler[0] = x;
		Euler[1] = y;
		Euler[2] = z;

	}

	//通过欧拉角计算对应的旋转矩阵
	void eulerToMatrix(Eigen::Vector3d& Euler, Eigen::Matrix3d &R)
	{
		// 计算旋转矩阵的X分量
		Eigen::Matrix3d R_x;
		R_x(0,0) = 1; R_x(0,1) = 0; R_x(0,2) = 0;
		R_x(1, 0) = 0; R_x(1, 1) = cos(Euler[0]); R_x(1,2) = -sin(Euler[0]);
		R_x(2,0) = 0; R_x(2,1) = sin(Euler[0]); R_x(2,2) = cos(Euler[0]);


		// 计算旋转矩阵的Y分量
		Eigen::Matrix3d R_y;
		R_y(0, 0) = cos(Euler[1]); R_y(0, 1) = 0; R_y(0, 2) = sin(Euler[1]);
		R_y(1, 0) = 0; R_y(1, 1) = 1; R_y(1, 2) = 0;
		R_y(2, 0) = -sin(Euler[1]); R_y(2, 1) = 0; R_y(2, 2) = cos(Euler[1]);


		// 计算旋转矩阵的Z分量
		Eigen::Matrix3d R_z;
		R_z(0, 0) = cos(Euler[2]); R_z(0, 1) = -sin(Euler[2]); R_z(0, 2) = 0;
		R_z(1, 0) = sin(Euler[2]); R_z(1, 1) = cos(Euler[2]); R_z(1, 2) = 0;
		R_z(2, 0) = 0; R_z(2, 1) = 0; R_z(2, 2) = 1;

		// 合并 
		R = R_z * R_y * R_x;
	}

	void HandEye::init_handeye(const CameraParam& cameraL, 
		const CameraParam& cameraR,
		const RigidMatrix& matL, const RigidMatrix& matR,
		const std::vector<Eigen::Vector3d>& robotPos, 
		const std::vector<ImageGreyb>& imageL,
		const std::vector<ImageGreyb>& imageR
		)
	{
		_cameraL.K = cameraL.K;
		_cameraL.dis = cameraL.dis;
		_cameraR.K = cameraR.K;
		_cameraR.dis = cameraR.dis;
		_matL = matL;
		_matR = matR;
		_matRL = _matL*_matR.inverse();
		_imageLb = imageL;
		_imageRb = imageR;
		_RosP = robotPos;

	}

	RetVal HandEye::show_edge(const ImageGreyb& imageL, const ImageGreyb& imageR, std::vector<EllipseResult>& ellipse_L, std::vector<EllipseResult>& ellipse_R)
	{
		if (imageL.get_height() <= 0 || imageL.get_width() <= 0 || imageR.get_height() <= 0 || imageR.get_width() <= 0)
		{
			std::cout << "the Input image is illegal" << std::endl;
			return RetVal_ILLEGAL_INPUT;
		}
		ImageGreyf imagel, imager;
		std::vector<EllipseEdge> ellipseL, ellipseR;
		image_conversion(imageL, imagel);
		image_conversion(imageR, imager);
		MarkPointRecognition Show;
		Show.set_canny_param(_imageParam.cannyParamH, _imageParam.cannyParamL, 1);
		Show.set_guass_param(_imageParam.gaussParam, _imageParam.gaussParam, 1);
		Show.set_ellipse_param(0.2, 3.0, _imageParam.ellipseL, _imageParam.ellipseH);// 参数可调

		auto runL = [&Show, &imagel, &ellipseL](){ Show.run(imagel, ellipseL, Canny_OpenCV); };
		auto runR = [&Show, &imager, &ellipseR](){ Show.run(imager, ellipseR, Canny_OpenCV); };

		std::thread calL(runL);
		std::thread calR(runR);
		calL.join();
		calR.join();

		if (ellipseL.size() >= 2 && ellipseR.size() >= 2)
		{
			//判定同心圆
			double x0 = 0.0;
			double y0 = 0.0;
			double x = 0.0;
			double y = 0.0;
			EllipseEdge ellipseTemp0, ellipseTemp1;
			std::vector<int> idL(2);
			std::vector<int> idR(2);
			for (int i = 0; i < ellipseL.size(); i++)
			{
				x0 = ellipseL[i].centerW;
				y0 = ellipseL[i].centerH;
				for (int j = i + 1; j < ellipseL.size(); j++)
				{
					x = ellipseL[j].centerW;
					y = ellipseL[j].centerH;
					if (abs(x - x0) < 2.0 && abs(y - y0) < 2.0)
					{
						idL[0] = i;
						idL[1] = j;
					}
				}
			}
			ellipseTemp0 = ellipseL[idL[0]];
			ellipseTemp1 = ellipseL[idL[1]];
			ellipseL.clear();
			ellipseL.push_back(ellipseTemp0);
			ellipseL.push_back(ellipseTemp1);

			for (int i = 0; i < ellipseR.size(); i++)
			{
				x0 = ellipseR[i].centerW;
				y0 = ellipseR[i].centerH;
				for (int j = i + 1; j < ellipseR.size(); j++)
				{
					x = ellipseR[j].centerW;
					y = ellipseR[j].centerH;
					if (abs(x - x0) < 2.0 && abs(y - y0) < 2.0)
					{
						idR[0] = i;
						idR[1] = j;
					}
				}
			}
			ellipseTemp0 = ellipseR[idR[0]];
			ellipseTemp1 = ellipseR[idR[1]];
			ellipseR.clear();
			ellipseR.push_back(ellipseTemp0);
			ellipseR.push_back(ellipseTemp1);
		}
		else
		{
			return RetVal_ILLEGAL_INPUT;
		}
		ellipse_L.resize(ellipseL.size());
		ellipse_R.resize(ellipseR.size());
		for (int i = 0; i < ellipseL.size(); ++i)
		{
			ellipse_L[i].angle = ellipseL[i].angle;
			ellipse_L[i].centerW = ellipseL[i].centerW;
			ellipse_L[i].centerH = ellipseL[i].centerH;
			ellipse_L[i].radiusA = ellipseL[i].radiusA;
			ellipse_L[i].radiusB = ellipseL[i].radiusB;
		}
		for (int i = 0; i < ellipseR.size(); ++i)
		{
			ellipse_R[i].angle = ellipseR[i].angle;
			ellipse_R[i].centerW = ellipseR[i].centerW;
			ellipse_R[i].centerH = ellipseR[i].centerH;
			ellipse_R[i].radiusA = ellipseR[i].radiusA;
			ellipse_R[i].radiusB = ellipseR[i].radiusB;
		}
		return RetVal_OK;
	}

	RetVal HandEye::extract_2dpoint(const std::vector<ImageGreyb>& imageLb, const std::vector<ImageGreyb>& imageRb, std::vector <Eigen::Vector2d>& pL, std::vector <Eigen::Vector2d>& pR)
	{
		if (imageLb.size() != 4 || imageRb.size() != 4)
		{
			std::cout << "the input image data is illegal!" << std::endl;
		}
		std::vector<ImageGreyf> imageLf, imageRf;
		ImageGreyf imageLfTemp, imageRfTemp;
		ImageGreyb imageLbTemp, imageRbTemp;

		MarkPointRecognition mark;
		std::vector <EllipseEdge> ellipseL, ellipseR;
		Eigen::Vector2d PL, PR;
		pL.clear();
		pR.clear();
		//set param and fit the ellipse
		mark.set_canny_param(_imageParam.cannyParamH, _imageParam.cannyParamL, 1);
		mark.set_guass_param(_imageParam.gaussParam, _imageParam.gaussParam, 1);
		mark.set_ellipse_param(0.2, 1.5, _imageParam.ellipseL, _imageParam.ellipseH);// 参数可调
		mark.set_CvCanny_param(_imageParam.cvCannyParamH, _imageParam.cvCannyParamL);
		for (size_t i = 0; i < imageLb.size(); i++)
		{
			imageLbTemp = imageLb[i];
			imageRbTemp = imageRb[i];
			image_conversion(imageLbTemp, imageLfTemp);
			image_conversion(imageRbTemp, imageRfTemp);
			if (mark.run(imageLfTemp, ellipseL,Canny_OpenCV) == RetVal_OK)
			{
				if (ellipseL.size() >= 1)
				{
					std::cout << "fit the left ellipse success!" << std::endl;
				}
				else
				{
					std::cout << "fit the left ellipse failed!" << std::endl;
					return RetVal_FINISHED;
				}
			}
			if (mark.run(imageRfTemp, ellipseR,Canny_OpenCV) == RetVal_OK)
			{
				if (ellipseR.size() >= 1)
				{
					std::cout << "fit the right ellipse success!" << std::endl;
				}
				else
				{
					std::cout << "fit the right ellipse failed!" << std::endl;
					return RetVal_FINISHED;
				}
			}
			//init the mark point in the left image and right image
			double xl = 0.0;
			double yl= 0.0;
			double xr = 0.0;
			double yr = 0.0;
			double xltemp = 0.0;
			double yltemp = 0.0;
			double xrtemp = 0.0;
			double yrtemp = 0.0;
			//compute the ellipse center
			for (int le = 0; le < ellipseL.size(); le++)
			{
				for (int i = le+1; i < ellipseL.size();i++)
				{
					xltemp = ellipseL[le].centerW;
					yltemp = ellipseL[le].centerH;

					if (abs(ellipseL[i].centerW-xltemp)<2.0 && abs(ellipseL[i].centerH-yltemp)<2.0)
					{
						xl = xltemp + ellipseL[i].centerW;
						yl = yltemp + ellipseL[i].centerH;

					}

				}

			}
			xl /= 2;
			yl /= 2;
			std::cout << "the left image center is::" << std::endl;
			std::cout << std::setprecision(10) << xl << "," << yl << std::endl;
			PL = { xl, yl };
			
			pL.push_back(PL);
			for (int r = 0; r < ellipseR.size(); r++)
			{
				for (int j = r+1; j < ellipseR.size(); j++)
				{
					xrtemp = ellipseR[r].centerW;
					yrtemp = ellipseR[r].centerH;

					if (abs(ellipseR[j].centerW - xrtemp) < 1.0 && abs(ellipseR[j].centerH - yrtemp) < 1.0)
					{
						xr = xrtemp + ellipseR[j].centerW;
						yr = yrtemp + ellipseR[j].centerH;
					}

				}
			}
			xr /= 2;
			yr /= 2;
			PR = { xr, yr };
			pR.push_back(PR);
			std::cout << "the right image center is::" << std::endl;
			std::cout << std::setprecision(10) << xr << "," << yr << std::endl;
		}
	
		return RetVal_OK;
	}
	
	RetVal HandEye::markpoint_reconstruct(const CameraCommonCaliParam& cameraL, const CameraCommonCaliParam& cameraR, const RigidMatrix& matRL,
		const std::vector<Eigen::Vector2d>& p2DL, const std::vector<Eigen::Vector2d>& p2DR, std::vector<Eigen::Vector3d>& output)
	{
		if (cameraL.K.size() != 5 || cameraL.dis.size() != 5 || cameraR.K.size() != 5 || cameraR.dis.size() != 5
			|| p2DL.size() != p2DR.size())
		{
			Sn_Error("illegal input");
			return RetVal_ILLEGAL_INPUT;
		}

		output.resize(p2DR.size());
		Eigen::Vector3d beg1, beg2, dir1, dir2;
		Eigen::Vector3d midTran, midEuler;
		Eigen::Matrix3d midRot;
		double lensum = 0;
		int num = 0;
		RigidMatrix matRLTemp;
		matRLTemp = matRL.inverse();
		for (int i = 0; i < p2DR.size(); ++i)
		{
			output[i].setConstant(0);
			if (p2DL[i][0] < 0 || p2DL[i][1] < 0 || p2DR[i][0] < 0 || p2DR[i][1] < 0) continue;
			Eigen::Vector2d tL, tR;
			cameraL.image_to_camera(p2DL[i], tL);
			cameraR.image_to_camera(p2DR[i], tR);
			
			beg1.setConstant(0);
			beg2 = matRL.get_translation();
			dir1[0] = tL[0]; dir1[1] = tL[1]; dir1[2] = 1.0;
			dir2[0] = tR[0]; dir2[1] = tR[1]; dir2[2] = 1.0;
			dir1.normalize();
			dir2 = matRL.transform_normal(dir2).normalized();

			Eigen::Vector3d posTemp,pos;
			double len = two_line_intersection(beg1, dir1, beg2, dir2, posTemp);
			midTran = matRLTemp.get_translation() / 2;
			midRot = matRLTemp.get_rotation();
			//旋转矩阵转欧拉角
			MatrixToEuler(midRot, midEuler);
			midEuler[0] = midEuler[0] / 2;
			midEuler[1] = midEuler[1] / 2;
			midEuler[2] = midEuler[2] / 2;
			//欧拉角转旋转矩阵
			eulerToMatrix(midEuler, midRot);
			pos = midRot*(posTemp + midTran);

			std::cout << len << std::endl;

			//std::cout << pos << std::endl;
			lensum += len;
			++num;
			output[i] = std::move(pos);
			
		}
		std::cout << p2DR.size() << std::endl;
		std::cout << lensum / num << std::endl;
		return RetVal_OK;
	}
	
	RetVal HandEye::is_point_valid(const std::vector <Eigen::Vector3d>& P)
	{
		Eigen::Vector3d p1, p2, p3;
		Eigen::Vector3d v1, v2, v12;
		double sinF;

		p1 = P[0];
		p2 = P[1];
		p3 = P[2];

		v1 = p1 - p2;
		v2 = p1 - p3;

		v12 = v1.cross(v2);
		sinF = v12.dot(v12) / (v1.dot(v1)*v2.dot(v2));
		double len1 = v1.norm();
		double len2 = v2.norm();
		std::cout << "the SinF is :" << sinF << std::endl;
		if (sinF < asin(15 / 180 * M_PI))
		{
			return RetVal_ILLEGAL_INPUT;
		}
		return RetVal_OK;
	}

	RetVal HandEye::pose_Estimation_3d3d(const std::vector<Eigen::Vector3d>& pts1, const std::vector<Eigen::Vector3d>& pts2, Eigen::Matrix<double, 3, 3>& R, Eigen::Matrix<double, 3, 1>& t)
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
		//Convert the reflection matrix into the rotation matrix
		if (R.determinant() < 0)
		{
			V(0, 2) = -1 * V(0, 2);
			V(1, 2) = -1 * V(1, 2);
			V(2, 2) = -1 * V(2, 2);
			//R = U*(V.transpose());
			R = V*(U.transpose());
		}
		t = Eigen::Vector3d(p1[0], p1[1], p1[2]) - R*Eigen::Vector3d(p2[0], p2[1], p2[2]);
		return RetVal_OK;
	}

	RetVal HandEye::handeye_error(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,double& error)
	{
		Eigen::Vector3d errorMat,p4;
		errorMat.setConstant(100);
		//compute the error of hand-eye calibration
		p4[0] = R(0, 0)*p1[0] + R(0, 1)*p1[1] + R(0, 2)*p1[2] + t[0];
		p4[1] = R(1, 0)*p1[0] + R(1, 1)*p1[1] + R(1, 2)*p1[2] + t[1];
		p4[2] = R(2, 0)*p1[0] + R(2, 1)*p1[1] + R(2, 2)*p1[2] + t[2];

		errorMat[0] = R(0, 0)*p1[0] + R(0, 1)*p1[1] + R(0, 2)*p1[2] + t[0] - p2[0];
		errorMat[1] = R(1, 0)*p1[0] + R(1, 1)*p1[1] + R(1, 2)*p1[2] + t[1] - p2[1];
		errorMat[2] = R(2, 0)*p1[0] + R(2, 1)*p1[1] + R(2, 2)*p1[2] + t[2] - p2[2];
		
		error = abs(errorMat[0])*abs(errorMat[0]) + abs(errorMat[1])*abs(errorMat[1]) + abs(errorMat[2])*abs(errorMat[2]);
		error = sqrt(error);
		return RetVal_OK;

	}

	RetVal HandEye::run_hand_eye(Eigen::Matrix4d& T, double& error)
	{
		if (_imageLb.size() != 4 || _imageRb.size() != 4)
		{
			std::cout << "the Input image is illegal" << std::endl;
			return RetVal_ILLEGAL_INPUT;
		}
		HandEye handEye;
		std::vector<Eigen::Vector2d> pL, pR;
		RetVal val1 = extract_2dpoint(_imageLb, _imageRb, pL, pR);
		if (val1 != RetVal_OK)
		{
			return val1;
		}
		std::vector<Eigen::Vector3d> scanPos;
		RetVal val2 = markpoint_reconstruct(_cameraL, _cameraR, _matRL, pL, pR, scanPos);
		if (val2 != RetVal_OK)
		{
			return val2;
		}
		//判断标定板共线和距离
		if (handEye.is_point_valid(scanPos) != RetVal_OK)
		{
			std::cout << "the calibTable position is not suitable: maybe too near between each other" << std::endl;
			return RetVal_ILLEGAL_INPUT;
		}

		if (_RosP.size() != 4)
		{
			std::cout << "the robot position input is illegal!" << std::endl;
			return RetVal_ILLEGAL_INPUT;
		}
		//判断机器人共线和距离
		if (handEye.is_point_valid(_RosP) != RetVal_OK)
		{
			std::cout << "the robot position is not suitable: maybe on the line or too near between each other" << std::endl;
			return RetVal_ILLEGAL_INPUT;
		}

		Eigen::Matrix3d R;
		Eigen::Vector3d t;
		RetVal val3 = pose_Estimation_3d3d(_RosP, scanPos, R, t);
		if (val3 != RetVal_OK)
		{
			return val3;
		}

		T.block(0, 0, 3, 3) = R.block(0, 0, 3, 3);
		T.block(0, 3, 3, 1) = t.block(0, 0, 3, 1);
		T(3, 0) = T(3, 1) = T(3, 2) = 0;
		T(3, 3) = 1;
		double errorTemp = 0.0;
		for (int i = 0; i < scanPos.size(); i++)
		{
			RetVal val4 = handEye.handeye_error(R, t, scanPos[i], _RosP[i], errorTemp);
			if (val4 != RetVal_OK)
			{
				return val4;
			}
			error += errorTemp;
		}
		error = error / scanPos.size();

		std::cout << "the hand eye calibration error: " << error << std::endl;
		return RetVal_OK;
	}
}


