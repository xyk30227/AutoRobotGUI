//#define GLOG_NO_ABBREVIATED_SEVERITIES
#ifndef SN3D_ALGORITHM_SN3D_ALGORITHMZYS_INCLUDE_H
#define SN3D_ALGORITHM_SN3D_ALGORITHMZYS_INCLUDE_H
#include "algorithmZysDefines.h"
namespace Sn3DAlgorithm
{
	/*机器人引导所用的四点法手眼标定算法*/
	//每次采集后调用此函数，保留数据供下一次判断；第一次采集时，id=0;第二次采集时，id=1;第三次采集时，id=2
	void SN3D_HANDEYE_API set_position(
		const int& id, 
		const Eigen::Vector3d& robotP,
		const std::vector<EllipseResult>& ellipse_L, 
		const std::vector<EllipseResult>& ellipse_R);

	//判断机器人位置是否合适
	//返回值为0，正常；返回值为-1，机器人位置错误；返回值为-2，标定板位置错误；返回值为1，运行错误；第二次采集后判断时，id=0;第三次采集后判断时，id=1；第四次采集后判断时，id=2；
	int SN3D_HANDEYE_API judge_position(
		const int& id, 
		const Eigen::Vector3d& currentRobotP,
		const std::vector<EllipseResult>& ellipse_L, 
		const std::vector<EllipseResult>& ellipse_R);

	/*供软件显示边缘所用*/
	RetVal SN3D_HANDEYE_API show_edge(
		const ImageGreyb& imageL, 
		const ImageGreyb& imageR,
		std::vector<EllipseResult>& ellipse_L, 
		std::vector<EllipseResult>& ellipse_R);//当前版本暂时不设置图像处理参数，使用默认值

	/*四点法求解eye to hand的手眼关系矩阵
	*@param[in]	    imageL:左相机图像
	*@param[in]	    imageR:右相机图像
	*@param[in]	    cameraL:左相机内参
	*@param[in]		cameraR:右相机内参
	*@param[in]		matL:左相机外参
	*@param[in]		matR:右相机外参
	*@param[in]		roPos:机器人tcp三维坐标
	*@param[out]	T:扫描仪相对于机器人基坐标系的刚体变换矩阵
	*@param[out]	error:手眼标定的误差
	*/
	RetVal SN3D_HANDEYE_API handeye_Cali(const std::vector<ImageGreyb>& imageL,
		const std::vector<ImageGreyb>& imageR, 
		const CameraParam& cameraL, 
		const CameraParam& cameraR,
		const RigidMatrix& matL,
		const RigidMatrix& matR,
		const std::vector<Eigen::Vector3d>& robotPos, 
		Eigen::Matrix4d& T, double& error);

	/**\ brief 四点法求解eye to hand手眼关系矩阵（直接点对）
	* @param[in]     robotPts:机器人坐标系下的点坐标
	* @param[in]     scannerPts:扫描仪坐标系下的坐标
	* @param[out]    mat:手眼关系矩阵
	* @param[out]    error:手眼标定误差(error=-1时:输入非法)
	*/
	RetVal SN3D_HANDEYE_API hand_eye_calibration_pts(
		const std::vector<Eigen::Vector3d>& robotPts,
		const std::vector<Eigen::Vector3d>& scannerPts,
		RigidMatrix& rt,
		double& error);

	/**\ brief 自动多点法求解eye to hand手眼关系矩阵
	* @param[in]     robotPts:机器人坐标系下的点坐标X/Y/Z/Rz(Rz为角度值)
	* @param[in]     scannerPts:扫描仪坐标系下的坐标
	* @param[out]    mat:手眼关系矩阵
	* @param[out]    error:手眼标定误差(error=-1时:输入非法)
	*/
	RetVal SN3D_HANDEYE_API auto_hand_eye_calibration(
		const std::vector<Eigen::Vector4d>& robotPts,
		const std::vector<Eigen::Vector3d>& scannerPts,
		Eigen::Matrix4d& T,
		double& error);

	/*机器人引导所用非线性优化的手眼标定算法：Eye_To_Hand和Eye_in_Hand
	*@param[in]	    imageL:左相机图像
	*@param[in]	    imageR:右相机图像
	*@param[in]	    cameraL:左相机内参
	*@param[in]	    cameraR:右相机内参
	*@param[in]	    matRL:右相机到左相机的刚体变换矩阵
	*@param[in]		robotPose:机器人末端姿态（末端坐标系到基坐标系下）
	*@param[in]		hway:手眼标定的方式
	*@param[out]	handEyeTrans:扫描仪到机械臂底座的刚体变换矩阵
	*@param[out]	error:手眼标定误差
	*/
	RetVal SN3D_HANDEYE_API robot_guide_calibration(
		const std::vector<ImageGreyb>& imageL, 
		const std::vector<ImageGreyb>& imageR, 
		const CameraParam& cameraL, 
		const CameraParam& cameraR,
		const RigidMatrix& matRL,
		const std::vector<RigidMatrix>& robotPose,
		const HandEyeWay&hway, RigidMatrix& handEyeTrans, double& error);

	/*机器人引导手眼标定板位置检测算法
	*@param[in]	    imageL:左相机拍摄图片
	*@param[in]		imageR:右相机拍摄图片
	*@param[in]	    cameraL:左相机内参
	*@param[in]		cameraR:右相机内参
	*@param[in]		matRL:左右相机关系
	*@param[out]	numPointL:左相机标志点识别数量, 当未识别到标定板时，值为-1；
	*@param[out]    numPointR:右相机标志点识别数量，当未识别到标定板时，值为-1；
	*@param[out]	centerCalibTable:标定板中心坐标(图像坐标X/Y和距离扫描仪高度Z)，当未识别到标定板时，值为(-1,-1,-1)
	*/
	RetVal SN3D_HANDEYE_API judge_robot_calib(
		const ImageGreyb& imageL, 
		const ImageGreyb& imageR, 
		const CameraParam& cameraL, 
		const CameraParam& cameraR,
		const RigidMatrix& matRL, 
		int& numPointL, 
		int& numPointR, 
		Eigen::Vector3d& centerCalibTable);

	/*机器人引导双目标定接口*/
	/*判定当前标定板是否处于合适位置以及自动曝光调节
	*@param[in]	    imageL:左相机拍摄图片
	*@param[in]		imageR:右相机拍摄图片
	*@param[out]	numPointL:左相机标志点识别数量, 当未识别到标定板时，值为-1；
	*@param[out]    numPointR:右相机标志点识别数量，当未识别到标定板时，值为-1；
	*@param[out]	centerCalibTable:标定板中心坐标，当未识别到标定板时，值为(-1,-1)
	*/
	RetVal SN3D_HANDEYE_API judge_calibTable(
		const ImageGreyb& imageL, 
		const ImageGreyb& imageR, 
		int& numPointL, 
		int& numPointR, 
		Eigen::Vector2d& centerCalibTable);

	/*扫描仪双目标定执行接口
	*@param[in]	    imageL:左相机拍摄图片
	*@param[in]		imageR:右相机拍摄图片
	*@param[in]	    leftCCF:左相机标定文件生成路径
	*@param[in]	    rightCCF:右相机标定文件生成路径
	*@param[in]	    encry:是否生成加密标定文件
	*@param[out]	error:标定残差
	*/
	RetVal SN3D_HANDEYE_API run_scanner_calibration(
		const std::vector<ImageGreyb>& imageL, 
		const std::vector<ImageGreyb>& imageR,
		const int& imgWidth, 
		const int& imgHeight,
		const std::vector<DistanceRef>& disRefs,
		const std::vector<std::string>& leftCCF, 
		const std::vector<std::string>& rightCCF,
		double& error, bool encry);

	/*机械臂补扫接口函数*/
	/*求解AXP，进行优化，eye in hand
	*@param[in]	    robotPose:机器人末端姿态（X / Y / Z / RX / RY / RZ）
	*@param[in]		p3D:标定板上标志点的三维坐标（相对于左相机）
	*@param[out]	handEyeTrans:机械臂末端到扫描仪的刚体变换矩阵
	*/
	RetVal SN3D_HANDEYE_API robot_scan_calibration(
		const std::vector<Eigen::Matrix<double, 3, 2>>& robotPose, 
		const std::vector<std::vector<Eigen::Vector3d>>& p3D, 
		RigidMatrix& handEyeTrans);

}//namespace

#endif //SN3D_ALGORITHM_SN3D_ALGORITHMZYS_INCLUDE_H