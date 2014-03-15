#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <math.h>

#define PI 3.1415926

// PCL general
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// PCL visualization
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

// PCL Filters
   
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h> 
// global to allow all parts of the program to access them and not be created each time
pcl::visualization::PCLVisualizer cloud_viewer ("HDT Cloud Viewer"); 

ros::Publisher pub_pose;

pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_1 (new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_2 (new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_3 (new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_Final (new pcl::PointCloud<pcl::PointXYZ>());

	geometry_msgs::PoseStamped box_pose;
        unsigned char red [20] = {255, 0, 0, 255, 255, 0,255, 0, 0, 255, 255, 0,255, 0, 0, 255, 255, 0,255,0};
      unsigned char grn [20] = { 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255,0,255};
      unsigned char blu [20] = { 0, 0, 255, 0, 255, 255, 0, 0, 255, 0, 255, 255, 0, 0, 255, 0, 255, 255,0,0};

  pcl::ModelCoefficients final_coefficients;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  char name[1024];
void cloud_cb(const pcl::PCLPointCloud2ConstPtr& ros_cloud)
{
		cloud_viewer.spinOnce(100);
		cloud_viewer.removeAllPointClouds();
		cloud_viewer.removeAllShapes();
		cloud_viewer.removeCoordinateSystem();
	if(ros::ok())
	{
		
		pcl::fromPCLPointCloud2(*ros_cloud,*PCL_1);
 // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> vox;
  vox.setInputCloud (PCL_1);
  vox.setLeafSize (0.01f, 0.01f, 0.01f);
  vox.filter (*PCL_2);

  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>()) ;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 

  // Set parameters
  mls.setInputCloud (PCL_2);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.09);

  // Reconstruct
  mls.process (*mls_points);
pcl::copyPointCloud (*mls_points, *PCL_2);


  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (PCL_2);
  sor.setMeanK (100);
  sor.setStddevMulThresh (0.5);
  sor.filter (*PCL_1);

  size_t i = 0, nr_points = (int) PCL_1->points.size ();
  // While 30% of the original cloud is still there
  pcl::PointXYZ finalCentroid;
  finalCentroid.x=0;
  finalCentroid.y=0;
  finalCentroid.z=0;
  float min_distance=10000000;
  while (PCL_1->points.size () > 0.1 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (PCL_1);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (PCL_1);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*PCL_2);
    if(PCL_2->points.size()>400)
{
    //std::cerr << "PointCloud representing the planar component"<<i<<": " << PCL_2->width * PCL_2->height << " data points." << std::endl;
           sprintf (name, "plane_%zu", i);
pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> color (PCL_2, red[i], grn[i], blu[i]);
    cloud_viewer.addPointCloud<pcl::PointXYZ> (PCL_2,color, name);
    *PCL_Final+=*PCL_2;
      	  pcl::PointXYZ centroid;
	  centroid.x = 0.0;
	  centroid.y = 0.0;
	  centroid.z = 0.0;
 
	  for(size_t i = 0; i < PCL_2->points.size(); i+=2){
	    centroid.x += PCL_2->points[i].x;
	    centroid.y += PCL_2->points[i].y;
	    centroid.z += PCL_2->points[i].z;
	  }
  
	  centroid.x /= PCL_2->points.size()/2;
	  centroid.y /= PCL_2->points.size()/2;
	  centroid.z /= PCL_2->points.size()/2; 
//	  std::cerr << "Centroid at x: " << centroid.x << " y: " << centroid.y << " z: " << centroid.z << endl;
	  float cur_distance = sqrt(centroid.x*centroid.x+centroid.y*centroid.y+centroid.z*centroid.z);
	  if (cur_distance<min_distance)
	  {
		finalCentroid.x=centroid.x;
		finalCentroid.y=centroid.y;
		finalCentroid.z=centroid.z;
		final_coefficients=*coefficients;
		min_distance=cur_distance;
	  }
}
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*PCL_3);
    PCL_1.swap (PCL_3);
    i++;
  }
		std::cout<<"Top Centroid  x: " << finalCentroid.x << " y: " << finalCentroid.y << " z: " << finalCentroid.z << endl;
		cloud_viewer.addSphere (finalCentroid, 0.05, 0.5, 0.5, 0.0, "sphere");
	    	cloud_viewer.addCoordinateSystem(0.5);
		cloud_viewer.setBackgroundColor (0,0,0);
		box_pose.pose.position.x=finalCentroid.x;
		box_pose.pose.position.y=finalCentroid.y;
		box_pose.pose.position.z=finalCentroid.z;
	          float den = sqrt(pow(final_coefficients.values[0],2)+pow(final_coefficients.values[1],2)+pow(final_coefficients.values[2],2));
                  float bz=cos(acos(final_coefficients.values[2]/den));
                  float by=cos(acos((final_coefficients.values[1])/den));
                  float bx=cos(acos(final_coefficients.values[0]/den)+PI/2);
                  float alpha=atan2(final_coefficients.values[1],final_coefficients.values[0]);
                  box_pose.pose.orientation.w=0;//cos(alpha/2);
                  box_pose.pose.orientation.x=sin(alpha/2)*(bx);
                  box_pose.pose.orientation.y=sin(alpha/2)*(by);
                  box_pose.pose.orientation.z=sin(alpha/2)*(bz);

		pub_pose.publish(box_pose);
	}

}

int main (int argc, char** argv) 
{ 

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);
	std::cout<<"Visualization Node Starting...."<<endl;
	
	ros::init(argc,argv,"hdt_visualization");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("input",1,cloud_cb);

	pub_pose = nh.advertise<geometry_msgs::PoseStamped>("output_pose",10);
	

	box_pose.header.frame_id="/camera_depth_optical_frame";
	std::cout<<"Visualization Node Started"<<endl;
	ros::spin();
	return (0); 
}
