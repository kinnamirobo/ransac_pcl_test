#include <iostream>  
#include <pcl/ModelCoefficients.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/visualization/cloud_viewer.h>  
  
int  
main (int argc, char** argv)  
{  
  pcl::PointCloud<pcl::PointXYZRGB> cloud;  
  
  //pcl::io::loadPCDFile<pcl::PointXYZRGB> ("ex1_all_0525.pcd", *cloud)
  
  pcl::io::loadPCDFile ("./ex1_all_0525.pcd", cloud);
  
  // Set a few outliers   

  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  
  // Create the segmentation object  
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;  
  // Optional  
  seg.setOptimizeCoefficients (true);  
  // Mandatory  
  seg.setModelType (pcl::SACMODEL_PLANE);  
  seg.setMethodType (pcl::SAC_RANSAC);  
  seg.setDistanceThreshold (0.1);  
  
  seg.setInputCloud (cloud.makeShared ());  
  seg.segment (*inliers, *coefficients);  
  
  if (inliers->indices.size () == 0)  
    {  
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");  
      return (-1);  
    }  
  
  std::cerr << "Model coefficients: " << coefficients->values[0] << " "  
   << coefficients->values[1] << " "  
   << coefficients->values[2] << " "  
   << coefficients->values[3] << std::endl;  
  
  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;  
  for (size_t i = 0; i < inliers->indices.size (); ++i) {  
    std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "  
     << cloud.points[inliers->indices[i]].y << " "  
     << cloud.points[inliers->indices[i]].z << std::endl;  
    cloud.points[inliers->indices[i]].r = 255;  
    cloud.points[inliers->indices[i]].g = 0;  
    cloud.points[inliers->indices[i]].b = 0;  
  }  
  pcl::visualization::CloudViewer viewer("Cloud Viewer");  
  
  viewer.showCloud(cloud.makeShared());  
  
  while (!viewer.wasStopped ())  
    {  
       
    }  
  
  return (0);  
}  

