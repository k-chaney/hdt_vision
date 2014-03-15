#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <math.h>

// PCL general
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// PCL visualization
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

// PCL Filters
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/statistical_outlier_removal.h>


#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

    
#include <pcl/kdtree/kdtree_flann.h> 
#include <pcl/surface/mls.h> 

#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/filters/extract_indices.h>

    
// global to allow all parts of the program to access them and not be created each time
pcl::visualization::PCLVisualizer cloud_viewer ("HDT Cloud Viewer"); 

ros::Publisher pub;

pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_1 (new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_2 (new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_Final (new pcl::PointCloud<pcl::PointXYZ>());

      unsigned char red [6] = {255, 0, 0, 255, 255, 0};
      unsigned char grn [6] = { 0, 255, 0, 255, 0, 255};
      unsigned char blu [6] = { 0, 0, 255, 0, 255, 255};

  // Init object (second point type is for the normals, even if unused)

      pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::Normal, pcl::Label> mps;
      std::vector<pcl::PlanarRegion<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZ> > > regions;
      pcl::PointCloud<pcl::PointXYZ>::Ptr contour (new pcl::PointCloud<pcl::PointXYZ>);
      size_t prev_models_size = 0;
      char name[1024];
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
   pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne; 

          pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
	geometry_msgs::PoseStamped box_pose;


void cloud_cb(const pcl::PCLPointCloud2ConstPtr& ros_cloud)
{
		cloud_viewer.spinOnce(100);
		cloud_viewer.removeAllPointClouds();
		cloud_viewer.removeAllShapes();
		cloud_viewer.removeCoordinateSystem();
	if(ros::ok())
	{
	        regions.clear();
		pcl::fromPCLPointCloud2(*ros_cloud,*PCL_1);

	        double normal_start = pcl::getTime ();
	          ne.setInputCloud (PCL_1);
	          ne.compute (*normal_cloud);
		double normal_end = pcl::getTime ();
          	std::cout << "Normal Estimation took " << double (normal_end - normal_start) << std::endl;

	        double plane_extract_start = pcl::getTime ();
		mps.setInputNormals(normal_cloud);
		mps.setInputCloud(PCL_1);
		mps.segmentAndRefine(regions);		
	        double plane_extract_end = pcl::getTime ();
        	std::cout << "Plane extraction took " << double (plane_extract_end - plane_extract_start) << std::endl;
         	std::cout << "Frame took " << double (plane_extract_end - normal_start) << std::endl;

		
		cloud_viewer.setBackgroundColor (0,0,0);
		cloud_viewer.addPointCloud<pcl::PointXYZ> (PCL_1, "hdt_cloud");
	  cout << "Regions size: " << regions.size()<<endl; 
	  box_pose.pose.position.x=0;
	  box_pose.pose.position.y=0;
	  box_pose.pose.position.z=0;
	  box_pose.pose.orientation.x=0;
	  box_pose.pose.orientation.y=0;
	  box_pose.pose.orientation.z=0;
          for (size_t i = 0; i < regions.size (); i++)
          {
            Eigen::Vector3f center = regions[i].getCentroid ();
            Eigen::Vector4f model = regions[i].getCoefficients ();
            pcl::PointXYZ pt1 = pcl::PointXYZ (center[0], center[1], center[2]);
            pcl::PointXYZ pt2 = pcl::PointXYZ (center[0] + (0.5f * model[0]),
                                               center[1] + (0.5f * model[1]),
                                               center[2] + (0.5f * model[2]));
            sprintf (name, "normal_%zu", i);
            cloud_viewer.addArrow (pt2, pt1, 1.0, 0, 0, false, name);
	    if (pt1.y < box_pose.pose.position.y)
	    {
	          box_pose.pose.position.x=pt1.x;
	          box_pose.pose.position.y=pt1.y;
	          box_pose.pose.position.z=pt1.z;
		  float den = sqrt(pow(pt2.x-pt1.x,2)+pow(pt2.y-pt1.y,2)+pow(pt2.z-pt1.z,2));
		  float bz=((pt2.z-pt1.z)/den);
	          float by=((pt2.y-pt1.y)/den);
	          float bx=cos(acos((pt2.x-pt1.x)/den)-3.1415926/2);
		  float alpha=atan2(pt2.y-pt1.y,pt2.x-pt1.x);
		  box_pose.pose.orientation.w=0;//cos(alpha/2);

		  box_pose.pose.orientation.x=sin(alpha/2)*(bx);
		  box_pose.pose.orientation.y=sin(alpha/2)*(by);
		  box_pose.pose.orientation.z=sin(alpha/2)*(bz);
            }
            contour->points = regions[i].getContour ();
            sprintf(name, "plane_%u", i);
            pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> color (contour, red[i], grn[i], blu[i]);

            cloud_viewer.addPointCloud (contour, color, name);
            cloud_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
	    cloud_viewer.addCoordinateSystem(0.5);
          }
		cout << box_pose << endl;
	if(box_pose.pose.position.z!=0)
	{
		pub.publish(box_pose);
	}
	}

}

int main (int argc, char** argv) 
{ 

  // Reconstruct
      ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
      ne.setMaxDepthChangeFactor (0.3f);
      ne.setNormalSmoothingSize (100.0f);

      mps.setMinInliers (10000);
      mps.setAngularThreshold (0.017453 * 2.0); //3 degrees
      mps.setDistanceThreshold (0.1); //3cm
	
	std::cout<<"Visualization Node Starting...."<<endl;
	
	ros::init(argc,argv,"hdt_visualization");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("input",1,cloud_cb);

	pub = nh.advertise<geometry_msgs::PoseStamped>("output",10);
	
	box_pose.header.frame_id="/camera_depth_optical_frame";
	std::cout<<"Visualization Node Started"<<endl;
	ros::spin();
	return (0); 
}
