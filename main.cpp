#include <librealsense2/rs.hpp> 
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <librealsense2/rs.hpp>     
#include <opencv2/opencv.hpp>
#include "pclFunc.cpp"
//#include "segmentation.cpp"
#include <unistd.h>
#include <pcl/common/io.h>
#include <pcl/common/copy_point.h>
#include <pcl/common/concatenate.h> 
#include <pcl/keypoints/harris_3d.h>
#include <pcl/filters/extract_indices.h>
#include <thread>

using namespace cv;
//#include <librealsense/rs.hpp>

pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto ptr = points.get_vertices();
	for (auto& p : cloud->points)
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}
    //VoxelGrid(cloud);
    //PassThrough(cloud);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::copyPointCloud(*VoxelGrid(PassThrough(cloud)), *c_cloud);
	return c_cloud;
}

int main(int argc, char * argv[]) try
{
   
    rs2::pipeline pipe;
    rs2::config cfg;
    const auto window_name = "color";
    namedWindow(window_name, WINDOW_AUTOSIZE);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8);
   // rs2::colorizer color_map;
   pipe.start(cfg);
    //rs2::pipeline_profile profile = p.start(cfg);
    //namedWindow("RGB", WINDOW_AUTOSIZE);
    //namedWindow("Depth", WINDOW_AUTOSIZE);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("regular pc"));
    //pcl::visualization::CloudViewer viewer("viewer");
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_cluster(new pcl::visualization::PCLVisualizer("extracted clusters"));
    viewer_cluster->setBackgroundColor(0, 0, 0);
   // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_bb(new pcl::visualization::PCLVisualizer("Bounding Boxes"));
    //viewer_bb->setRepresentationToWireframeForAllActors();
    
    
    
    while(!viewer->wasStopped()){
        
        
        
        
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frame depth = frames.get_depth_frame();
        rs2::frame color_Frame = frames.get_color_frame();
        Mat _color(Size(640, 480), CV_8UC3, (void*)color_Frame.get_data(), Mat::AUTO_STEP);
        //Mat _depth(Size(640, 480), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
        //imshow("Depth",depthFrame);
        //imshow("Segmented",segmentation(colorFrame, 16));
       //Mat _color(Size(640, 480), CV_8UC3, (void*)color_Frame.get_data(), Mat::AUTO_STEP);
        //cv::Size size = depthFrame.size();
        imshow("color", _color);
		

        rs2::pointcloud pc;
        auto points = pc.calculate(depth);
        
       
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        ///pcl::copyPointCloud<pcl::PointXYZRGB>(*points_to_pcl(points), *cloud);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = points_to_pcl(points);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered,"cloud1");
        std::vector<pcl::PointIndices> cluster_indices = planeSeg(cloud_filtered);

       //viewer_bb->initCameraParameters();
	    //viewer_bb->setRepresentationToWireframeForAllActors();
        //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> key(harris3d(points_to_pcl(points)),0,255,0);

        
        //viewer.addPointCloud<pcl::PointXYZ>(harris3d(points_to_pcl(points)), key,"cloud2");
       //viewer_bb->spinOnce(10);
        
        viewer_cluster->spinOnce(10);
        viewer->spinOnce(10);
        //imshow("RGB",colorFrame);
         //imshow("color", _color);
        //viewer->removeAllShapes();
        viewer->removeAllPointClouds();
        //viewer.showCloud(points_to_pcl(points));   
        int j = 0;
        waitKey(1);
        
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    
    viewer_cluster->removeAllShapes();
    viewer_cluster->removeAllPointClouds();
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
	  for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		  cloud_cluster->points.push_back(cloud_filtered->points[*pit]); 
	  cloud_cluster->width = cloud_cluster->points.size();
	  cloud_cluster->height = 1;
	  cloud_cluster->is_dense =  false;
	  pcl::PointXYZ minPt, maxPt;

	  pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);
	  std::cout << "Max x: " << maxPt.x << std::endl;
	  std::cout << "Max y: " << maxPt.y << std::endl;
	  std::cout << "Max z: " << maxPt.z << std::endl;
	  std::cout << "Min x: " << minPt.x << std::endl;
	  std::cout << "Min y: " << minPt.y << std::endl;
	  std::cout << "Min z: " << minPt.z << std::endl;
	  float x_min = minPt.x;
	  float x_max = maxPt.x;
	  float y_min = minPt.y;
	  float y_max = maxPt.y;
	  float z_min = minPt.z;
	  float z_max = maxPt.z;

	  viewer_cluster->setRepresentationToWireframeForAllActors();
	  viewer_cluster->addCube(x_min, x_max, y_min, y_max, z_min, z_max, 2.0, 0, 0, std::to_string(j+50), 0);

	  viewer_cluster->addPointCloud<pcl::PointXYZ>(cloud_cluster, std::to_string(j));
	  viewer_cluster->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, std::to_string(j));
	  j++;
      
  }

    }
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
