#pragma once

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

/**
  * @file config.h
  * 配置PCL中的点类型及相关指针
  * @ingroup PCL_Tools
  */

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef PointCloudT::Ptr PointCloudPtr;

typedef std::pair<std::string, std::vector<float>> VFH_Hist;