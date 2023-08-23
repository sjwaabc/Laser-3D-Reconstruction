#include "StitchPoint.h"

void StitchPoint::Stitch(std::vector<std::string> & basis_strDir, std::vector<std::string> & laser_strDir)
{

	std::vector	<cv::Mat> Rot;
	std::vector	<cv::Mat> Tvec;


	// 
	std::vector<cv::Mat> debug_Rot_r;
	std::vector<cv::Mat> debug_Tvec_r;
	cv::Mat debug_R0_ = cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat debug_T0_ = cv::Mat::zeros(3, 1, CV_64FC1);
	debug_Rot_r = { debug_R0_ };
	debug_Tvec_r = { debug_T0_ };


	std::vector	<cv::Mat> Rot_r;
	std::vector	<cv::Mat> Tvec_r;

	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(3, 3, 0.0235, 0.01, dictionary);



	cv::Mat objPoints_aruco, temp(4, 1, CV_32FC3);
	temp.copyTo(objPoints_aruco);

	objPoints_aruco.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
	objPoints_aruco.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
	objPoints_aruco.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
	objPoints_aruco.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);

	for (int kk = 0; kk < basis_strDir.size(); kk++) 
	{
		cv::Mat image, imageCopy;
		image = cv::imread(basis_strDir[kk], cv::IMREAD_GRAYSCALE);
		image.copyTo(imageCopy);

		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f> > corners;
		
		cv::aruco::detectMarkers(image, dictionary, corners, ids);
		std::vector<std::vector<cv::Point2f> > corners1;

		for (int kk = 0; kk < corners.size(); kk++) {
			std::vector<cv::Point2f> temp;
			undistortedP(corners[kk], temp);
			corners1.push_back(temp);
		}
		

		// if at least one marker detected
		if (ids.size() > 0) 
		{
			cv::aruco::drawDetectedMarkers(imageCopy, corners1, ids);
			cv::Vec3d rvec, tvec;
			int valid = estimatePoseBoard(corners1, ids, board, intrinsic_laser, distortion_laser, rvec, tvec);
			// if at least one board marker detected
			if (valid > 0) 
			{
				cv::aruco::drawAxis(imageCopy, intrinsic_laser, distortion_laser, rvec, tvec, 0.05);
				cv::Mat r, t, _r;
				r = (cv::Mat)rvec;
				t = (cv::Mat)tvec;
				cv::Rodrigues(r, _r);
				Rot.push_back(_r);
				Tvec.push_back(t);
			} // end-if
			else {
				aruco_errror_index.push_back(kk);
			}
				
		} // end-if
		else {
			aruco_errror_index.push_back(kk);
		}

	} //end-for aruco

	std::cout << "from aruco to cameraN transform matrix done." << std::endl;

	std::vector <std::vector<cv::Point3d>> objPoints;

	
	for (int kk = 0; kk < laser_strDir.size(); kk++) 
	{
		if (count(aruco_errror_index.begin(), aruco_errror_index.end(), kk))
		{
			cout << "aruco_errror_index found" << endl;
		}
		else
		{
			cout << "aruco_errror_index NOT found" << endl;
			std::vector<cv::Point2d> pointln, _pointln;

			getLaserCenterPoints(laser_strDir[kk], pointln);

			if (pointln.size() > 5) {

				cv::undistortPoints(pointln, _pointln, intrinsic_laser, distortion_laser);

				std::vector<cv::Point3d> _objPoints;


				// calculate intersection points between the obj_plane and all the  3d points (w * dst.x, w * dst.y, w)
				for (size_t i = 0; i < _pointln.size(); ++i) {
					double w = -laser_plane[3] / (laser_plane[0] * _pointln[i].x + laser_plane[1] * _pointln[i].y + laser_plane[2]);
					_objPoints.push_back(-(cv::Point3f(w * _pointln[i].x, w * _pointln[i].y, w)));
				}

				objPoints.push_back(_objPoints);
			}
			else {
				laser_errror_index.push_back(kk);
			}
			
		}
		

	} // end-for

	std::cout << "Laser points done." << std::endl;

	for (int kk = 1; kk < Rot.size(); kk++) {

		if (count(laser_errror_index.begin(), laser_errror_index.end(), kk))
		{
			cout << "laser_errror_index found" << endl;
		}
		else
		{
			cout << "laser_errror_index NOT found" << endl;
			cv::Mat proj1 = cv::Mat::zeros(3, 4, CV_32FC1);
			cv::Mat proj2 = cv::Mat::zeros(3, 4, CV_32FC1);

			Rot[0].convertTo(proj1(cv::Range(0, 3), cv::Range(0, 3)), CV_32FC1);
			Tvec[0].convertTo(proj1.col(3), CV_32FC1);

			cv::Mat R_T = Rot[kk].t();
			cv::Mat T_T = -(R_T * Tvec[kk]);

			R_T.convertTo(proj2(cv::Range(0, 3), cv::Range(0, 3)), CV_32FC1);
			T_T.convertTo(proj2.col(3), CV_32FC1);


			cv::Mat _proj1 = cv::Mat::zeros(4, 4, CV_32FC1);
			cv::Mat _proj2 = cv::Mat::zeros(4, 4, CV_32FC1);
			cv::Mat _col = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
			_col.convertTo(_proj1.col(3), CV_32FC1);
			_col.convertTo(_proj2.col(3), CV_32FC1);

			proj1.convertTo(_proj1(cv::Range(0, 3), cv::Range(0, 4)), CV_32FC1);
			proj2.convertTo(_proj2(cv::Range(0, 3), cv::Range(0, 4)), CV_32FC1);


			cv::Mat C1Cn = _proj1 * _proj2;
			cv::Mat sub_R = C1Cn.rowRange(0, 3).colRange(0, 3);
			cv::Mat sub_T = C1Cn.rowRange(0, 3).colRange(3, 4);


			Rot_r.push_back(sub_R);
			Tvec_r.push_back(sub_T);

			// debug
			debug_Rot_r.push_back(sub_R);
			debug_Tvec_r.push_back(sub_T);

		}

	} // end-for
	

	// debug
	save_structure("../structure.yml", debug_Rot_r, debug_Tvec_r);

	std::cout << "from cameraN to camera1 transform matrix done." << std::endl;

	std::vector<cv::Point3d> Points;
	for (int kk = 1; kk < objPoints.size(); kk++) 
	{
		for (int jj = 0; jj < objPoints[kk].size(); jj++) 
		{

			cv::Mat point = (cv::Mat_<float>(3, 1) << objPoints[kk][jj].x, objPoints[kk][jj].y, objPoints[kk][jj].z);
			cv::Mat __point = Rot_r[kk - 1] * point;
			__point += Tvec_r[kk - 1];


			cv::Mat floatMat;
			retRotation.convertTo(floatMat, CV_32F);
			cv::Mat temp = floatMat * __point;
			Points.push_back(-(cv::Point3d(temp.at<float>(0, 0), temp.at<float>(1, 0), temp.at<float>(2, 0))));

		} // end-for
		
	} // end-for

	WritePointCloudPly("Points.ply", Points);

	std::cout << "from cameraN to camera1 points done." << std::endl;

} // end-fuc


void StitchPoint::BackLasersFromimg(std::vector<std::string> & basis_strDir, std::vector<std::string> & laser_strDir, std::vector<cv::Point3d>& obj_pts)
{
	std::vector	<cv::Mat> rvecslist;
	std::vector	<cv::Mat> tvecslist;

	std::vector<vector<cv::Point2f>> image_points_seq; 

	cv::Size board_size = cv::Size(qipan_rows, qipan_cols);    


	cv::Size imgSize;


	for (int i = 0; i < basis_strDir.size(); ++i)
	{

		cv::Mat image = cv::imread(basis_strDir[i], cv::IMREAD_GRAYSCALE);
		cv::Mat imageCopy;

		imgSize.width = image.cols;
		imgSize.height = image.rows;

		std::vector<cv::Point2f> image_points_buf; 

		// debug
		{

			if (0 == findChessboardCorners(image, board_size, image_points_buf))
			{

				return ;

			} // end-if
			else
			{

				cornerSubPix(image, image_points_buf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));

				find4QuadCornerSubpix(image, image_points_buf, cv::Size(qipan_rows, qipan_cols)); /
				image_points_seq.push_back(image_points_buf); 

				drawChessboardCorners(image, board_size, image_points_buf, true); 
				imshow("Camera Calibration", image);
				cv::waitKey(500);     

			} // end-else


		} // end debug

	} // end-for


	cv::Size2d square_size = cv::Size2d(qipan_single_size, qipan_single_size);  
	vector<vector<cv::Point3f>> object_points; 
	vector<int> point_counts;


	int i, j, t;
	for (t = 0; t < basis_strDir.size(); t++)
	{
		vector<cv::Point3f> tempPointSet;
		for (j = 0; j < board_size.height; j++)
		{
			for (i = 0; i < board_size.width; i++)
			{

				cv::Point3f realPoint;

				//realPoint.x = i * square_size.width;
				//realPoint.y = j * square_size.height;	
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	} // end-for



	for (i = 0; i < basis_strDir.size(); i++)
	{
		point_counts.push_back(board_size.width * board_size.height);
	}


	for (int ii = 0; ii < object_points.size(); ii++) {
		cv::Vec3d rvec_, tvec_;
		// PnP
		cv::solvePnP(object_points[ii], image_points_seq[ii], intrinsic_laser, distortion_laser, rvec_, tvec_, cv::SOLVEPNP_ITERATIVE);

		cv::Mat rvec = (cv::Mat)rvec_;
		cv::Mat tvec = (cv::Mat)tvec_;
		rvecslist.push_back(rvec);
		tvecslist.push_back(tvec);
	} // end-for


	std::vector<cv::Point2d> pointln;
	std::vector < std::vector<cv::Point2d> > pointsln;
	
	
	for (int kk = 0; kk < laser_strDir.size(); kk++) 
	{

		getLaserCenterPoints(laser_strDir[kk], pointln);

		BackProjection(pointln, rvecslist[kk], tvecslist[kk], obj_pts);

		// debug
		pointsln.push_back(pointln);

		pointln.clear();
	}


	for (int ii = 0; ii < obj_pts.size(); ii++) {
		obj_pts[ii].x = obj_pts[ii].x *1000.0;
		obj_pts[ii].y = obj_pts[ii].y *1000.0;
		obj_pts[ii].z = obj_pts[ii].z *1000.0;
	}



	// debug
	WritePointCloudPly("../obj_pts.ply", obj_pts);

	cv::Vec4d plane;
	std::vector < cv::Point3d >  retpoint;
	Fit3dPlane("../", obj_pts, plane, retpoint);

	// debug
	normal[0] = plane[0];
	normal[1] = plane[1];
	normal[2] = plane[2];
	laser_plane = plane;

	//cv::Mat retRotPoints; 
	std::vector<cv::Point3d> retRotPoints;

	CalcRotation(retpoint, retRotation, retRotPoints);

	WritePointCloudPly("../points1.ply", retRotPoints);


	int k = 0;
	k++;

	std::vector<cv::Point3d> __obj_pts;
	for (int kk = 0; kk < obj_pts.size(); kk++)
	{
		cv::Mat point = (cv::Mat_<double>(1, 3) << obj_pts[kk].x, obj_pts[kk].y, obj_pts[kk].z);
		cv::Mat new_point = point * retRotation;
		__obj_pts.push_back(cv::Point3d(new_point.at<double>(0, 0), new_point.at<double>(0, 1), new_point.at<double>(0, 2)));

	} // end-for

	WritePointCloudPly("../_obj_pts.ply", __obj_pts);

	// debug
	{

		std::vector<cv::Point3d> test_pts;
		std::vector<cv::Point2d> temp;
		for (int rr = 0; rr < pointsln.size(); rr++)
		{
			cv::undistortPoints(pointsln[rr], temp, intrinsic_laser, distortion_laser);
			// calculate intersection points between the obj_plane and all the  3d points (w * dst.x, w * dst.y, w)
			for (size_t i = 0; i < temp.size(); ++i) {
				double w = -laser_plane[3] / (laser_plane[0] * temp[i].x + laser_plane[1] * temp[i].y + laser_plane[2]);
				test_pts.push_back(cv::Point3f(w * temp[i].x, w * temp[i].y, w));
			}
			temp.clear();
		}

		WritePointCloudPly("../test_pts.ply", test_pts);
	}

	

} // end-fuc


void StitchPoint::getLaserCenterPoints(std::string & strDir, std::vector<cv::Point2d> & result)
{
	cv::Mat image;

	image = cv::imread(strDir, cv::IMREAD_GRAYSCALE);
	if (image.empty())
	{
		return ;
	}

	cv::Mat newImage = cv::Mat::zeros(image.size(), image.type());


	for (int kk = 0; kk < image.rows; kk++)
	{
		for (int jj = 0; jj < image.cols; jj++) {

			if (image.at<uchar>(kk, jj) > 230) {
				newImage.at<uchar>(kk, jj) = 255;
			}

		} // end-for
	} // end-for

	cv::Mat __newImage = cv::Mat::zeros(image.size(), image.type());

	for (size_t i = 0; i < newImage.cols; i++)
	{
		int sum = 0; int num = 0;
		for (size_t j = 0; j < newImage.rows; j++)
		{
			if (newImage.at<uchar>(j, i) == 255)
			{
				sum += j;
				num++;
			}
		}
		if (num == 0)
			continue;
		cv::Point2d temp(i, 1.0*sum / num);
		__newImage.at<uchar>(1.0*sum / num, i) = 255;
		

		result.push_back(temp);

	} // end-for


} // end-fuc


void StitchPoint::CalcRotation(std::vector<cv::Point3d> points, cv::Mat & rotation, std::vector<cv::Point3d> &pointsR)
{

	cv::Point3d p1 = points[0];
	cv::Point3d p2 = points[1];
	cv::Point3d p3 = points[2];

	cv::Point3d plane_norm_;
	plane_norm_ = (p2 - p1).cross(p2 - p3);

	double plane_norm_1 = plane_norm_.x*plane_norm_.x + plane_norm_.y*plane_norm_.y + plane_norm_.z*plane_norm_.z;
	double plane_norm_2 = std::sqrt(plane_norm_1);
	plane_norm_ = plane_norm_ / plane_norm_2;


	cv::Point3d plane_norm(normal[0], normal[1], normal[2]);


	cv::Point3d xz_norm(0.0, 1.0, 0.0);
	std::cout << "plane_norm: " << plane_norm << std::endl;
	std::cout << "xz_norm: " << xz_norm << std::endl;

	double v1v2 = plane_norm.dot(xz_norm);

	double v1_norm = plane_norm.x*plane_norm.x + plane_norm.y*plane_norm.y + plane_norm.z*plane_norm.z;
	double v2_norm = xz_norm.x*xz_norm.x + xz_norm.y*xz_norm.y + xz_norm.z*xz_norm.z;

	double theta = std::acos(v1v2 / (std::sqrt(v1_norm)*std::sqrt(v2_norm)));


	cv::Point3d axis_v1v2 = xz_norm.cross(plane_norm);


	double v1v2_2 = axis_v1v2.x*axis_v1v2.x + axis_v1v2.y*axis_v1v2.y + axis_v1v2.z*axis_v1v2.z;
	double v1v2_n = std::sqrt(v1v2_2);
	axis_v1v2 = axis_v1v2 / v1v2_n;
	std::cout << "axis_v1v2: " << axis_v1v2 << std::endl;
	std::cout << "theta: " << theta << std::endl;


	cv::Point3d axis_v1v2_cv = -theta * axis_v1v2;
	std::cout << "axis_v1v2_cv: " << axis_v1v2_cv << std::endl;

	cv::Mat R_vec = (cv::Mat_<double>(3, 1) << axis_v1v2_cv.x, axis_v1v2_cv.y, axis_v1v2_cv.z);

	cv::Rodrigues(R_vec, rotation);
	//std::cout << " rotation  CV::" << rotation << std::endl;
	//std::vector<cv::Point3f> points1;
	cv::Mat afterPt;
	//Mat afterPt1 = (Mat_<double>(3, 1) << 1, 1, 1);
	//Mat afterPt2 = (Mat_<double>(3, 1) << 1, 1, 1);
	cv::Mat points2;
	for (size_t i = 0; i < points.size(); i++)
	{
		cv::Mat Pt = (cv::Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z);
		//std::cout << "orignal     " << Pt << std::endl    ;
		afterPt = rotation * Pt;
		double x = afterPt.at<double>(0, 0);
		double y = afterPt.at<double>(1, 0);
		double z = afterPt.at<double>(2, 0);
		pointsR.push_back(cv::Point3f(x, y, z));
		//std::cout << "orignal     " << afterPt << std::endl;
		//std::cout << "orignal     " << afterPt1 << std::endl;
	   // hconcat(afterPt1, afterPt, afterPt1);
	   // hconcat(afterPt2, Pt, afterPt2);
	}
	//points1 = afterPt1.t();
	//pointsR = afterPt2.t();
	//std::cout << "orignal     " << points1 << std::endl;

} // end-fuc


void StitchPoint::BackProjection(std::vector<cv::Point2d> &im_pts, cv::Mat &rvec, cv::Mat &tvec, std::vector<cv::Point3d> &obj_pts)
{
	std::vector<cv::Point2f> src, dst;

	cv::Mat(im_pts).copyTo(src);

	cv::undistortPoints(src, dst, intrinsic_laser, distortion_laser);
	cv::Vec4d obj_plane;
	GetObjectPlaneInCameraCoordinate(rvec, tvec, obj_plane);

	// calculate intersection points between the obj_plane and all the  3d points (w * dst.x, w * dst.y, w)
	for (size_t i = 0; i < dst.size(); ++i) {
		double w = -obj_plane[3] / (obj_plane[0] * dst[i].x + obj_plane[1] * dst[i].y + obj_plane[2]);
		obj_pts.push_back(cv::Point3f(w * dst[i].x, w * dst[i].y, w));
	}
}


void StitchPoint::GetObjectPlaneInCameraCoordinate(cv::Mat &rvec, cv::Mat &tvec, cv::Vec4d &plane)
{

	// construct the [R | t] matrix for the plane
	cv::Mat rot, rigid;
	cv::Rodrigues(rvec, rot);
	cv::hconcat(rot, tvec, rigid); 


	cv::Mat origin = cv::Mat::zeros({ 1,4 }, CV_64F);
	cv::Vec4d dd;
	origin.at<double>(3, 0) = 1.0;
	origin = rigid * origin;
	cv::Mat unit_z = cv::Mat::zeros({ 1,4 }, CV_64F);
	unit_z.at<double>(2, 0) = 1.0;
	unit_z.at<double>(3, 0) = 1.0;
	unit_z = rigid * unit_z;
	unit_z = unit_z - origin;

	plane[0] = unit_z.at<double>(0, 0);
	plane[1] = unit_z.at<double>(1, 0);
	plane[2] = unit_z.at<double>(2, 0);
	plane[3] = -(plane[0] * origin.at<double>(0, 0) + plane[1] * origin.at<double>(1, 0) + plane[2] * origin.at<double>(2, 0));

} //end-fuc

void StitchPoint::Fit3dPlane(std::string Dir, const std::vector<cv::Point3d> &points, cv::Vec4d &plane, std::vector< cv::Point3d > & retp)
{
	std::vector<cv::Mat> pts;
	cv::split(points, pts);
	cv::Mat coords;
	cv::vconcat(pts, coords);
	coords = coords.t();
	cv::Mat centroid;
	cv::reduce(coords, centroid, 0, cv::REDUCE_AVG, CV_64F);
	cv::Mat tmp;
	cv::Mat normalized = coords - cv::repeat(centroid, coords.rows, 1);

	cv::Mat w, u, vt;
	cv::SVDecomp(normalized, w, u, vt);

	plane[0] = vt.at<double>(2, 0);
	plane[1] = vt.at<double>(2, 1);
	plane[2] = vt.at<double>(2, 2);
	plane[3] = -(plane[0] * centroid.at<double>(0, 0) + plane[1] * centroid.at<double>(0, 1) + plane[2] * centroid.at<double>(0, 2));

	// write plane ply file
	double z0 = 0.0;
	double x0 = 0.0;
	double y0 = -plane[3] / plane[1];

	double z1, z2;
	z1 = z2 = centroid.at<double>(0, 2) * 1.2;

	cv::Mat minv, maxv;
	cv::reduce(coords, minv, 0, cv::REDUCE_MIN, CV_64F);
	cv::reduce(coords, maxv, 0, cv::REDUCE_MAX, CV_64F);
	cv::Mat range = maxv - minv;

	double x1, y1, x2, y2;
	if (range.at<double>(0, 0) > range.at<double>(0, 1)) {
		x1 = minv.at<double>(0, 0);
		y1 = -(plane[0] * x1 + plane[2] * z1 + plane[3]) / plane[1];
		x2 = maxv.at<double>(0, 0);
		y2 = -(plane[0] * x2 + plane[2] * z2 + plane[3]) / plane[1];
	}
	else {
		y1 = minv.at<double>(0, 0);
		x1 = -(plane[1] * y1 + plane[2] * z1 + plane[3]) / plane[0];
		y2 = maxv.at<double>(0, 0);
		x2 = -(plane[1] * y2 + plane[2] * z2 + plane[3]) / plane[0];
	}

	std::vector<cv::Point3d> laser;
	laser.push_back(cv::Point3d(x0, y0, z0));
	laser.push_back(cv::Point3d(x1, y1, z1));
	laser.push_back(cv::Point3d(x2, y2, z2));

	retp.push_back(laser[0]);
	retp.push_back(laser[1]);
	retp.push_back(laser[2]);

	WritePlanePly(Dir + "../laser.ply", laser);

} // end-fuc



template <typename T>
void StitchPoint::undistortedP(T &P, std::vector<cv::Point2f> &P_f_UNDISTORT)
{

	for (int kk = 0; kk < P.size(); kk++) {
		P[kk].x += 0;
		P[kk].y += 0;
	} // end-for

	std::vector<cv::Point2f> Temps;
	cv::Mat Temps_K = (cv::Mat_<double>(3, 3) <<
		1., 0., 1., 0.,
		1., 1., 0., 0., 1.);

	if (std::is_same<T, std::vector<cv::Point>>::value) {
		std::vector<cv::Point2f> P_f(P.size());

		// ת���� std::vector<cv::Point2f>
		std::transform(P.begin(), P.end(), P_f.begin(),
			[](const cv::Point& p)
		{
			return cv::Point2f(static_cast<float>(p.x), static_cast<float>(p.y));
		}); // end-lamda


#ifdef CV_VERSION_7
		cv::undistortImagePoints(P_f, P_f_UNDISTORT, intrinsic_laser, distortion_laser);
#endif // CV_VERSION_7

		cv::undistortPoints(P_f, P_f_UNDISTORT, intrinsic_laser, distortion_laser);
		// version < 4.7.0 
		for (int i = 0; i < P_f_UNDISTORT.size(); i++) {
			cv::Mat p(3, 1, CV_64FC1);
			p.at<double>(0, 0) = P_f_UNDISTORT[i].x;
			p.at<double>(1, 0) = P_f_UNDISTORT[i].y;
			p.at<double>(2, 0) = 1;

			cv::Mat new_p = intrinsic_laser * p;
			P_f_UNDISTORT[i].x = new_p.at<double>(0, 0);
			P_f_UNDISTORT[i].y = new_p.at<double>(1, 0);

		} // end-for

	} // end-if
	else
	{



#ifdef CV_VERSION_7
		cv::undistortImagePoints(P, P_f_UNDISTORT, intrinsic_laser, distortion_laser);
#endif // CV_VERSION_7

		cv::undistortPoints(P, P_f_UNDISTORT, intrinsic_laser, distortion_laser);
		// version < 4.7.0 
		for (int i = 0; i < P_f_UNDISTORT.size(); i++) {
			cv::Mat p(3, 1, CV_64FC1);
			p.at<double>(0, 0) = P_f_UNDISTORT[i].x;
			p.at<double>(1, 0) = P_f_UNDISTORT[i].y;
			p.at<double>(2, 0) = 1;

			cv::Mat new_p = intrinsic_laser * p;
			P_f_UNDISTORT[i].x = new_p.at<double>(0, 0);
			P_f_UNDISTORT[i].y = new_p.at<double>(1, 0);

		} // end-for


	} // end-else


} // end-fuction



void StitchPoint::WritePointCloudPly(const cv::String &file, const std::vector<cv::Point3d> &points) {
	std::ofstream ply;
	ply.open(file);

	// header
	ply << "ply" << std::endl;
	ply << "format ascii 1.0" << std::endl;
	ply << "element vertex " << points.size() << std::endl;
	ply << "property float x" << std::endl;
	ply << "property float y" << std::endl;
	ply << "property float z" << std::endl;
	ply << "end_header" << std::endl;

	ply << std::setprecision(4);
	for (size_t i = 0; i < points.size(); ++i) {
		ply << points[i].x << " " << points[i].y << " " << points[i].z << std::endl;
	}

	ply.close();
}


void StitchPoint::WritePlanePly(const cv::String &file, const std::vector<cv::Point3d> &plane) {
	std::ofstream ply;
	ply.open(file);

	ply << "ply" << std::endl;
	ply << "format ascii 1.0" << std::endl;
	ply << "element vertex " << plane.size() << std::endl;
	ply << "property float x" << std::endl;
	ply << "property float y" << std::endl;
	ply << "property float z" << std::endl;
	ply << "property uchar red" << std::endl;
	ply << "property uchar green" << std::endl;
	ply << "property uchar blue" << std::endl;
	ply << "property uchar alpha" << std::endl;
	ply << "element face 1" << std::endl;
	ply << "property list uchar int vertex_indices" << std::endl;
	ply << "end_header" << std::endl;

	ply << std::setprecision(4);
	for (size_t i = 0; i < plane.size(); ++i) {
		ply << plane[i].x << " " << plane[i].y << " " << plane[i].z << " 255 0 0 100" << std::endl;
	}
	ply << "3 0 1 2" << std::endl;
	ply.close();
}

void StitchPoint::save_structure(string file_name, vector<cv::Mat>& rotations, vector<cv::Mat>& motions)
{
	int n = (int)rotations.size();

	cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
	fs << "Camera Count" << n;
	fs << "Point Count" << (int)rotations.size();

	fs << "Rotations" << "[";
	for (size_t i = 0; i < n; ++i)
	{
		fs << rotations[i];
	}
	fs << "]";

	fs << "Motions" << "[";
	for (size_t i = 0; i < n; i++)
	{
		fs << motions[i];
	}
	fs << "]";

	fs.release();

}


