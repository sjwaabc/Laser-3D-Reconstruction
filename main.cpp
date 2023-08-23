#include "StitchPoint.h"
#include "viz_together.h"

int main() {
	// viz
	m_viz *viz = new m_viz(1);

	const int numbers = 20;

	StitchPoint sp;
	
	// path
	std::vector<std::string> images_basis;
	std::vector<std::string> images_laser;
	std::vector<std::string> __images_basis;
	std::vector<std::string> __images_laser;


	images_basis.push_back(sp.m_basis_folder + "chess0000.bmp");
	images_basis.push_back(sp.m_basis_folder + "chess0001.bmp");
	images_basis.push_back(sp.m_basis_folder + "chess0002.bmp");
	images_basis.push_back(sp.m_basis_folder + "chess0003.bmp");

	images_laser.push_back(sp.m_laser_folder + "laser0000.bmp");
	images_laser.push_back(sp.m_laser_folder + "laser0001.bmp");
	images_laser.push_back(sp.m_laser_folder + "laser0002.bmp");
	images_laser.push_back(sp.m_laser_folder + "laser0003.bmp");

	for (int kk = 1; kk <= numbers; kk++) {
		
		std::string name = "";
		if (kk <= 9) { 
			name = sp.m_folder_aruco + "aruco_" + std::to_string(kk) + ".bmp";
		}
		else
		{
			name = sp.m_folder_aruco + "aruco_" + std::to_string(kk) + ".bmp";
		}
		__images_basis.push_back(name);
	}
	


	for (int kk = 1; kk <= numbers; kk++) {
		std::string name = "";
		if (kk <= 9) {
			name = sp.m_folder_laser + "laser_" + std::to_string(kk) + ".bmp";
		}
		else
		{
			name = sp.m_folder_laser + "laser_" + std::to_string(kk) + ".bmp";
		}
		__images_laser.push_back(name);
	}

	
	std::vector<cv::Point3d> obj_pts;


	sp.BackLasersFromimg(images_basis, images_laser, obj_pts);

	// 
	sp.Stitch(__images_basis, __images_laser);


	cv::waitKey(0);

	return 0;
}