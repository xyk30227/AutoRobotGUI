//#define GLOG_NO_ABBREVIATED_SEVERITIES
#ifndef SN3D_ALGORITHM_SN3D_ALGORITHMZYS_INCLUDE_H
#define SN3D_ALGORITHM_SN3D_ALGORITHMZYS_INCLUDE_H
#include "algorithmZysDefines.h"
namespace Sn3DAlgorithm
{
	/*�������������õ��ĵ㷨���۱궨�㷨*/
	//ÿ�βɼ�����ô˺������������ݹ���һ���жϣ���һ�βɼ�ʱ��id=0;�ڶ��βɼ�ʱ��id=1;�����βɼ�ʱ��id=2
	void SN3D_HANDEYE_API set_position(
		const int& id, 
		const Eigen::Vector3d& robotP,
		const std::vector<EllipseResult>& ellipse_L, 
		const std::vector<EllipseResult>& ellipse_R);

	//�жϻ�����λ���Ƿ����
	//����ֵΪ0������������ֵΪ-1��������λ�ô��󣻷���ֵΪ-2���궨��λ�ô��󣻷���ֵΪ1�����д��󣻵ڶ��βɼ����ж�ʱ��id=0;�����βɼ����ж�ʱ��id=1�����Ĵβɼ����ж�ʱ��id=2��
	int SN3D_HANDEYE_API judge_position(
		const int& id, 
		const Eigen::Vector3d& currentRobotP,
		const std::vector<EllipseResult>& ellipse_L, 
		const std::vector<EllipseResult>& ellipse_R);

	/*�������ʾ��Ե����*/
	RetVal SN3D_HANDEYE_API show_edge(
		const ImageGreyb& imageL, 
		const ImageGreyb& imageR,
		std::vector<EllipseResult>& ellipse_L, 
		std::vector<EllipseResult>& ellipse_R);//��ǰ�汾��ʱ������ͼ���������ʹ��Ĭ��ֵ

	/*�ĵ㷨���eye to hand�����۹�ϵ����
	*@param[in]	    imageL:�����ͼ��
	*@param[in]	    imageR:�����ͼ��
	*@param[in]	    cameraL:������ڲ�
	*@param[in]		cameraR:������ڲ�
	*@param[in]		matL:��������
	*@param[in]		matR:��������
	*@param[in]		roPos:������tcp��ά����
	*@param[out]	T:ɨ��������ڻ����˻�����ϵ�ĸ���任����
	*@param[out]	error:���۱궨�����
	*/
	RetVal SN3D_HANDEYE_API handeye_Cali(const std::vector<ImageGreyb>& imageL,
		const std::vector<ImageGreyb>& imageR, 
		const CameraParam& cameraL, 
		const CameraParam& cameraR,
		const RigidMatrix& matL,
		const RigidMatrix& matR,
		const std::vector<Eigen::Vector3d>& robotPos, 
		Eigen::Matrix4d& T, double& error);

	/**\ brief �ĵ㷨���eye to hand���۹�ϵ����ֱ�ӵ�ԣ�
	* @param[in]     robotPts:����������ϵ�µĵ�����
	* @param[in]     scannerPts:ɨ��������ϵ�µ�����
	* @param[out]    mat:���۹�ϵ����
	* @param[out]    error:���۱궨���(error=-1ʱ:����Ƿ�)
	*/
	RetVal SN3D_HANDEYE_API hand_eye_calibration_pts(
		const std::vector<Eigen::Vector3d>& robotPts,
		const std::vector<Eigen::Vector3d>& scannerPts,
		RigidMatrix& rt,
		double& error);

	/**\ brief �Զ���㷨���eye to hand���۹�ϵ����
	* @param[in]     robotPts:����������ϵ�µĵ�����X/Y/Z/Rz(RzΪ�Ƕ�ֵ)
	* @param[in]     scannerPts:ɨ��������ϵ�µ�����
	* @param[out]    mat:���۹�ϵ����
	* @param[out]    error:���۱궨���(error=-1ʱ:����Ƿ�)
	*/
	RetVal SN3D_HANDEYE_API auto_hand_eye_calibration(
		const std::vector<Eigen::Vector4d>& robotPts,
		const std::vector<Eigen::Vector3d>& scannerPts,
		Eigen::Matrix4d& T,
		double& error);

	/*�������������÷������Ż������۱궨�㷨��Eye_To_Hand��Eye_in_Hand
	*@param[in]	    imageL:�����ͼ��
	*@param[in]	    imageR:�����ͼ��
	*@param[in]	    cameraL:������ڲ�
	*@param[in]	    cameraR:������ڲ�
	*@param[in]	    matRL:�������������ĸ���任����
	*@param[in]		robotPose:������ĩ����̬��ĩ������ϵ��������ϵ�£�
	*@param[in]		hway:���۱궨�ķ�ʽ
	*@param[out]	handEyeTrans:ɨ���ǵ���е�۵����ĸ���任����
	*@param[out]	error:���۱궨���
	*/
	RetVal SN3D_HANDEYE_API robot_guide_calibration(
		const std::vector<ImageGreyb>& imageL, 
		const std::vector<ImageGreyb>& imageR, 
		const CameraParam& cameraL, 
		const CameraParam& cameraR,
		const RigidMatrix& matRL,
		const std::vector<RigidMatrix>& robotPose,
		const HandEyeWay&hway, RigidMatrix& handEyeTrans, double& error);

	/*�������������۱궨��λ�ü���㷨
	*@param[in]	    imageL:���������ͼƬ
	*@param[in]		imageR:���������ͼƬ
	*@param[in]	    cameraL:������ڲ�
	*@param[in]		cameraR:������ڲ�
	*@param[in]		matRL:���������ϵ
	*@param[out]	numPointL:�������־��ʶ������, ��δʶ�𵽱궨��ʱ��ֵΪ-1��
	*@param[out]    numPointR:�������־��ʶ����������δʶ�𵽱궨��ʱ��ֵΪ-1��
	*@param[out]	centerCalibTable:�궨����������(ͼ������X/Y�;���ɨ���Ǹ߶�Z)����δʶ�𵽱궨��ʱ��ֵΪ(-1,-1,-1)
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

	/*����������˫Ŀ�궨�ӿ�*/
	/*�ж���ǰ�궨���Ƿ��ں���λ���Լ��Զ��ع����
	*@param[in]	    imageL:���������ͼƬ
	*@param[in]		imageR:���������ͼƬ
	*@param[out]	numPointL:�������־��ʶ������, ��δʶ�𵽱궨��ʱ��ֵΪ-1��
	*@param[out]    numPointR:�������־��ʶ����������δʶ�𵽱궨��ʱ��ֵΪ-1��
	*@param[out]	centerCalibTable:�궨���������꣬��δʶ�𵽱궨��ʱ��ֵΪ(-1,-1)
	*/
	RetVal SN3D_HANDEYE_API judge_calibTable(
		const ImageGreyb& imageL, 
		const ImageGreyb& imageR, 
		int& numPointL, 
		int& numPointR, 
		Eigen::Vector2d& centerCalibTable);

	/*ɨ����˫Ŀ�궨ִ�нӿ�
	*@param[in]	    imageL:���������ͼƬ
	*@param[in]		imageR:���������ͼƬ
	*@param[in]	    leftCCF:������궨�ļ�����·��
	*@param[in]	    rightCCF:������궨�ļ�����·��
	*@param[in]	    encry:�Ƿ����ɼ��ܱ궨�ļ�
	*@param[out]	error:�궨�в�
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

	/*��е�۲�ɨ�ӿں���*/
	/*���AXP�������Ż���eye in hand
	*@param[in]	    robotPose:������ĩ����̬��X / Y / Z / RX / RY / RZ��
	*@param[in]		p3D:�궨���ϱ�־�����ά���꣨������������
	*@param[out]	handEyeTrans:��е��ĩ�˵�ɨ���ǵĸ���任����
	*/
	RetVal SN3D_HANDEYE_API robot_scan_calibration(
		const std::vector<Eigen::Matrix<double, 3, 2>>& robotPose, 
		const std::vector<std::vector<Eigen::Vector3d>>& p3D, 
		RigidMatrix& handEyeTrans);

}//namespace

#endif //SN3D_ALGORITHM_SN3D_ALGORITHMZYS_INCLUDE_H