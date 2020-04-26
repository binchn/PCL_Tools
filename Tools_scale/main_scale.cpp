#include "config.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/visualization/pcl_visualizer.h>


/**
  * @file main_scale.cpp
  * ��������ƽ������ű任
  * @ingroup PCL_Tools
  */


int
main(int argc, char** argv)
{
	float scale = 0.001; // ����ϵ����scaleС��1��Ϊ��С������1��Ϊ�Ŵ�
	pcl::console::parse_argument(argc, argv, "-scale", scale);
	pcl::console::print_value("scale = %f \n", scale);
	
	// PCD�ļ�·����
	std::string dir_name = "models_pcd";
	pcl::console::parse_argument(argc, argv, "-dir", dir_name);
	boost::filesystem::path base_dir = "E:\\PCL_Projects\\pcd_files\\" + dir_name;
	// ��ȡԴ����
	std::string cloud_name = "sat.pcd";
	pcl::console::parse_argument(argc, argv, "-source", cloud_name);
	std::string source_file_name = base_dir.string() + "\\" + cloud_name;
	PointCloudPtr cloud_src(new PointCloudT);
	pcl::io::loadPCDFile(source_file_name, *cloud_src);
	pcl::console::print_info("loaded pcd file : %s \n", source_file_name.c_str());
	
	// ���ű任
	PointCloudPtr cloud_scale(new PointCloudT);
	Eigen::Matrix4f scale_mat; // �任����
	scale_mat.setIdentity(4,4); // ��λ����
	scale_mat(0, 0) = scale;
	scale_mat(1, 1) = scale;
	scale_mat(2, 2) = scale;
	pcl::transformPointCloud(*cloud_src, *cloud_scale, scale_mat);
	pcl::console::print_info("transform point cloud with matrix : \n");
	std::cout << scale_mat << std::endl;

	// �������
	std::string output_name = source_file_name.substr(0, source_file_name.rfind(".")) + "_scaled.pcd";
	pcl::console::parse_argument(argc, argv, "-target", output_name);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scale_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud_scale, *cloud_scale_xyz);
	pcl::io::savePCDFile(output_name, *cloud_scale_xyz);
	pcl::console::print_info("save scaled pcd file : %s \n", output_name.c_str());
}