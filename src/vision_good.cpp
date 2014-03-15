#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// for writing pcd files
#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/statistical_outlier_removal.h>


#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/organized_multi_plane_segmentation.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include <time.h>

using namespace std;
ros::Publisher pub;
typedef pcl::PointXYZ rgbpoint;
typedef pcl::PointCloud<pcl::PointXYZ> cloudrgb;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	cloudrgb::Ptr PC (new cloudrgb());
	cloudrgb::Ptr PC_filtered (new cloudrgb());
	pcl::fromROSMsg(*cloud, *PC_filtered); //Now you can process this PC using the pcl functions 
	sensor_msgs::PointCloud2 cloud_filtered;
	if(PC_filtered->width!=0&&PC_filtered->height!=0)
	{

		  // Reduces for calculation speed
	   pcl::VoxelGrid<pcl::PointXYZ> vox ;
	   vox.setInputCloud (PC_filtered);
	   vox.setLeafSize (0.01, 0.01, 0.01);
	   vox.filter (*PC);

	   pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	   sor.setInputCloud(PC);
	   sor.setMeanK(50);
	   sor.setStddevMulThresh (0.05);
	   sor.filter(*PC_filtered);

	     // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (PC_filtered);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.process (mls_points);


	    cout << "saved" << endl;
	    pcl::io::savePCDFile("/home/kenneth/yolo/trial.pcd", mls_points);
	}
}

int
main (int argc, char** argv)
{

  cout<< "Starting"<<endl;
  // Initialize ROS

  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points_xyzrgb", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  cout << "ROS Started" << endl;
  // Spin
  ros::spin ();
}
