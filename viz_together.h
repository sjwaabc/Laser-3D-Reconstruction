#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

class m_viz {
public:
	m_viz(int mode){
		if (mode == 0) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::io::loadPLYFile("Points.ply", *cloud);
			pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
			viewer.addPointCloud(cloud, "cloud");
			viewer.spin();
		}
		if (mode == 1) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::io::loadPLYFile("../laser.ply", *cloud1);
			pcl::io::loadPLYFile("../obj_pts.ply", *cloud2);
			pcl::io::loadPLYFile("../points1.ply", *cloud3);
			pcl::io::loadPLYFile("Points.ply", *cloud4);
			*cloud = *cloud1;
			*cloud += *cloud2;
			*cloud += *cloud3;
			*cloud += *cloud4;
			pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
			viewer.addPointCloud(cloud, "cloud");
			viewer.spin();
		}

	}

};
