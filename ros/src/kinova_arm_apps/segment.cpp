#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <mir_3d_image_segmentation/Clusters.h>
#include <mir_3d_image_segmentation/Cluster_Pixels.h>
#include <mir_3d_image_segmentation/Pixel_Coord.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher cloud_pub;
ros::Publisher clusters_pub;
bool publish_output_pc;
std::string output_pc_frame;

void cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input)
{
    //Downsample by x3
    int scale = 3;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_downsampled->width = input->width / scale;
    cloud_downsampled->height = input->height / scale;
    cloud_downsampled->points.resize(cloud_downsampled->width * cloud_downsampled->height);
    for( size_t i = 0, ii = 0; i < cloud_downsampled->height; ii += scale, i++)
    {
        for( size_t j = 0, jj = 0; j < cloud_downsampled->width; jj += scale, j++)
        {
            cloud_downsampled->at(j, i) = input->at(jj, ii); //at(column, row)
        }
    }

    //Remove points further than 1m away from the camera in z-direction
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_downsampled);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_through(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter (*cloud_pass_through);

    //Voxel grid filter for uniform pointcloud
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_pass_through);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter (*cloud_filtered);

    mir_3d_image_segmentation::Clusters Clusters;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        mir_3d_image_segmentation::Cluster_Pixels Cluster_Pixels;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cloud_cluster->points.push_back (cloud_downsampled->points[*pit]); //*
            mir_3d_image_segmentation::Pixel_Coord Pixel_Coord;
            Pixel_Coord.y = (*pit / cloud_downsampled->width);
            Pixel_Coord.x = (*pit % cloud_downsampled->width);
            Cluster_Pixels.cluster_pixels.push_back(Pixel_Coord);
        }
        Clusters.clusters.push_back(Cluster_Pixels);
    }  
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    //std::cerr << "cluster inliers: " << cloud_cluster->width << std::endl;

    if (publish_output_pc)
    {
        sensor_msgs::PointCloud2 output;
        //pcl::toROSMsg(*segmented_cloud, output);
        pcl::toROSMsg(*cloud_cluster, output);
        output.header.frame_id = output_pc_frame;
        output.header.stamp = ros::Time::now();

        // Publish the cloud data
        cloud_pub.publish (output);
    }

    // Publish the cluster data
    pcl_conversions::fromPCL(input->header.stamp, Clusters.header.stamp);
    Clusters.height = cloud_downsampled->height;
    Clusters.width = cloud_downsampled->width;
    Clusters.scale = scale;
    clusters_pub.publish (Clusters);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pcl_segmentation");
    ros::NodeHandle nh("~");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > ("input", 1, cloud_cb);

    nh.param<bool>("publish_output_pc", publish_output_pc, "false");
    nh.param<std::string>("output_pc_frame", output_pc_frame, "");

    if (publish_output_pc)
    {
        // Create a ROS publisher for the output point cloud
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    }

    // Create a ROS publisher for the output point cloud
    clusters_pub = nh.advertise<mir_3d_image_segmentation::Clusters> ("clusters", 1);

    // Spin
    ros::spin ();
}
