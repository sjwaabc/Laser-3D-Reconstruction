#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/aruco.hpp>

namespace cvfs = cv::utils::fs;
using namespace std;

class StitchPoint
{
public:

	// laser plane
	void BackLasersFromimg(std::vector<std::string> & basis_strDir, std::vector<std::string> & laser_strDir, std::vector<cv::Point3d>& obj_pts);
	//
	void Stitch(std::vector<std::string> & basis_strDir, std::vector<std::string> & laser_strDir);

	// source path
	const std::string m_folder_aruco = "../images_stitch/aruco/";
	const std::string m_folder_laser = "../images_stitch/laser/";
	const std::string m_laser_folder = "../images_laserplane/";
	const std::string m_basis_folder = "../images_laserplane/";

private :

	// Backprojection
	void BackProjection(std::vector<cv::Point2d> &im_pts, cv::Mat &rvec, cv::Mat &tvec, std::vector<cv::Point3d> &obj_pts);

	// Singular value decomposition fitting surface
	void Fit3dPlane(std::string Dir, const std::vector<cv::Point3d> &points, cv::Vec4d &plane, std::vector< cv::Point3d > & retp);

	// Obtain the calibration board pose in the camera coordinate system
	void GetObjectPlaneInCameraCoordinate(cv::Mat &rvec, cv::Mat &tvec, cv::Vec4d &plane);

	// Rotate the plane parallel to the camera coordinate system XOZ plane and invert it
	void CalcRotation(std::vector<cv::Point3d> points, cv::Mat & rotation, std::vector<cv::Point3d> &pointsR);

	// Extracting Median Value of Laser Centerline Method
	void getLaserCenterPoints(std::string & strDir, std::vector<cv::Point2d> & result);

	// Distortion removal
	template <typename T>
	void undistortedP(T &P, std::vector<cv::Point2f> &P_f_UNDISTORT);

	// 
	void WritePointCloudPly(const cv::String &file, const std::vector<cv::Point3d> &points);
 
	//
	void WritePlanePly(const cv::String &file, const std::vector<cv::Point3d> &plane);
	
	//
	void save_structure(string file_name, vector<cv::Mat>& rotations, vector<cv::Mat>& motions);

	// Number of corners in a chessboard column
	const int qipan_rows = 11;
	const int qipan_cols = 9;
	const double qipan_single_size = 0.006f;	// m

	// Physical size of aruco code
	const double markerLength = 0.007f;	// m	

	const cv::Mat intrinsic_laser = (cv::Mat_<double>(3, 3) <<
		3.3454688908597482e+03, 0., 1.5355000000000000e+03, 0.,
		3.3485116997377795e+03, 1.0235000000000000e+03, 0., 0., 1.);

	const cv::Mat distortion_laser = (cv::Mat_<double>(14, 1) <<
		1.4474941088254642e-01, 1.2962021941297939e+01,
		-7.0062236999610757e-04, 2.5219921209614582e-02,
		-2.5423918359766316e+02, 1.9877836290817980e-01,
		1.3437147572330046e+01, -2.5860769426867284e+02,
		-2.5185238218198892e-02, -1.8856349102475420e-03,
		1.4765555440212105e-03, -4.6231821351718893e-03,
		4.9324761551408514e-03, 5.3149307524365995e-02);
	

	cv::Vec3d normal;

	cv::Vec4d laser_plane;

	cv::Mat retRotation;


	// Invalid frame
	std::vector<int> aruco_errror_index;
	std::vector<int> laser_errror_index;
};

