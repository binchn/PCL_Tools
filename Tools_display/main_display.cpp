#include "config.h"
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>

/**
  * @file main_display.cpp
  * ��ʾ������������ά��Χ��xyz�ߴ�
  * @ingroup PCL_Tools
  */


int
main(int argc, char** argv)
{
	// PCD�ļ�·����
	std::string dir_name = "models_pcd";
	pcl::console::parse_argument(argc, argv, "-dir", dir_name);
	boost::filesystem::path base_dir = "E:\\PCL_Projects\\pcd_files\\" + dir_name;
	// PCD�ļ���
	std::string source_cloud_name = "source_cloud.pcd";
	pcl::console::parse_argument(argc, argv, "-source", source_cloud_name);

	// ��ȡPCD�ļ�
	std::string source_file_name = base_dir.string() + "\\" +source_cloud_name;
	PointCloudPtr source_cloud(new PointCloudT);
	pcl::io::loadPCDFile(source_file_name, *source_cloud);

	// ��ʾPCD����
	pcl::visualization::PCLVisualizer viewer_source("viewer_source");
	viewer_source.addPointCloud(source_cloud, "source_cloud");
	viewer_source.setBackgroundColor(255, 255, 255); // ���ð�ɫ����
	viewer_source.addText(source_cloud_name, 20, 10, 1, 0, 0, source_cloud_name, 0); // Ĭ��idͬsource_cloud_name, Ĭ��viewerportΪall
	viewer_source.addCoordinateSystem(1, "global", 0);
	
	// ����3D��Χ��
	PointT min_p, max_p;
	pcl::getMinMax3D(*source_cloud, min_p, max_p);
	PointT p1, p2, p3, p4, p5, p6;
	p1.x = min_p.x; p1.y = min_p.y; p1.z = max_p.z;
	p2.x = min_p.x; p2.y = max_p.y; p2.z = max_p.z;
	p3.x = min_p.x; p3.y = max_p.y; p3.z = min_p.z;
	p4.x = max_p.x; p4.y = min_p.y; p4.z = max_p.z;
	p5.x = max_p.x; p5.y = min_p.y; p5.z = min_p.z;
	p6.x = max_p.x; p6.y = max_p.y; p6.z = min_p.z;

	viewer_source.addLine(min_p, p1, 0, 1, 0, "l_1", 0);
	viewer_source.addLine(min_p, p3, 0, 1, 0, "l_2", 0);
	viewer_source.addLine(min_p, p5, 0, 1, 0, "l_3", 0);
	viewer_source.addLine(p1, p2, 0, 1, 0, "l_4", 0);
	viewer_source.addLine(p1, p4, 0, 1, 0, "l_5", 0);
	viewer_source.addLine(p2, p3, 0, 1, 0, "l_6", 0);
	viewer_source.addLine(p2, max_p, 0, 1, 0, "l_7", 0);
	viewer_source.addLine(p3, p6, 0, 1, 0, "l_8", 0);
	viewer_source.addLine(p6, max_p, 0, 1, 0, "l_9", 0);
	viewer_source.addLine(p6, p5, 0, 1, 0, "l_10", 0);
	viewer_source.addLine(p5, p4, 0, 1, 0, "l_11", 0);
	viewer_source.addLine(p4, max_p, 0, 1, 0, "l_12", 0);
	
	// �����߿�
	viewer_source.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "l_1", 0);
	viewer_source.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "l_2", 0);
	viewer_source.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "l_3", 0);
	viewer_source.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "l_4", 0);
	viewer_source.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "l_5", 0);
	viewer_source.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "l_6", 0);
	viewer_source.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "l_7", 0);
	viewer_source.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "l_8", 0);
	viewer_source.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "l_9", 0);
	viewer_source.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "l_10", 0);
	viewer_source.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "l_11", 0);
	viewer_source.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "l_12", 0);
	
	// ��ʾ����xyz�ߴ�
	float x_len = max_p.x - min_p.x;
	float y_len = max_p.y - min_p.y;
	float z_len = max_p.z - min_p.z;
	std::string x_str = "x: " + std::to_string(x_len);
	std::string y_str = "y: " + std::to_string(y_len);
	std::string z_str = "z: " + std::to_string(z_len);
	viewer_source.addText(x_str, 20, 40, 0, 1, 0);
	viewer_source.addText(y_str, 20, 30, 0, 1, 0);
	viewer_source.addText(z_str, 20, 20, 0, 1, 0);

	// ��ʾ��������
	int size = source_cloud->points.size();
	std::string size_str = "points.size : " + std::to_string(size);
	viewer_source.addText(size_str, 20, 50, 0, 0, 1);

	// ���������С
	viewer_source.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 2, x_str, 0);
	viewer_source.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 2, y_str, 0);
	viewer_source.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 2, z_str, 0);

	// ������ʾ
	viewer_source.spin();
	while (!viewer_source.wasStopped())
	{
		viewer_source.spinOnce(100);
	}


}



