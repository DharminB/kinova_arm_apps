#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/norms.h>
#include <Eigen/Eigenvalues>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <Eigen/Eigenvalues>

ros::Publisher cloud_pub;
ros::Publisher pose_pub;
ros::Publisher event_out_pub;
bool publish_output_pc;
bool listening;
std::string output_pc_frame;
float z_threshold;
float x_threshold;

geometry_msgs::PoseStamped estimatePose(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_input_cloud)
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*xyz_input_cloud, centroid);

    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*xyz_input_cloud, centroid, covariance);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();

    // swap largest and second largest eigenvector so that y-axis aligns with largest eigenvector and z with the second largest
    eigen_vectors.col(0).swap(eigen_vectors.col(2));
    eigen_vectors.col(1) = eigen_vectors.col(2).cross(eigen_vectors.col(0));

    Eigen::Matrix4f eigen_vector_transform(Eigen::Matrix4f::Identity());
    eigen_vector_transform.block<3, 3>(0, 0) = eigen_vectors.transpose();
    eigen_vector_transform.block<3, 1>(0, 3) = -(eigen_vector_transform.block<3, 3>(0, 0) * centroid.head<3>());

    // transform cloud to eigenvector space
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(*xyz_input_cloud, transformed_cloud, eigen_vector_transform);

    // find mean diagonal
    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D(transformed_cloud, min_point, max_point);
    Eigen::Vector3f mean_diag = (max_point.getVector3fMap() + min_point.getVector3fMap()) / 2.0;

    // orientation and position of bounding box of cloud
    Eigen::Quaternionf orientation(eigen_vectors);
    Eigen::Vector3f position = eigen_vectors * mean_diag + centroid.head<3>();

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = position(0);
    pose_stamped.pose.position.y = position(1);
    pose_stamped.pose.position.z = position(2);
    pose_stamped.pose.orientation.w = orientation.w();
    pose_stamped.pose.orientation.x = orientation.x();
    pose_stamped.pose.orientation.y = orientation.y();
    pose_stamped.pose.orientation.z = orientation.z();

    return pose_stamped;
}

void cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input)
{
    if (!listening)
    {
        return;
    }
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
    pass.setFilterLimits (0.0, z_threshold);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_through(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter (*cloud_pass_through);

    //Voxel grid filter for uniform pointcloud
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_pass_through);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter (*cloud_filtered);

    //Estimate most dominant plane coefficients
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    //std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

    //Project plane model inliers to plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> project_inliers;
    project_inliers.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    project_inliers.setInputCloud(cloud_filtered);
    project_inliers.setModelCoefficients(coefficients);
    project_inliers.setIndices(inliers);
    project_inliers.setCopyAllData(false);
    project_inliers.filter(*plane);

    //Compute plane convex hull
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> convex_hull;
    convex_hull.setInputCloud(plane);
    convex_hull.reconstruct(*hull);

    //Extract points inside polygonal prism of plane convex hull
    pcl::PointIndices::Ptr segmented_cloud_inliers(new pcl::PointIndices);
    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> extract_polygonal_prism;
    extract_polygonal_prism.setInputPlanarHull(hull);
    extract_polygonal_prism.setInputCloud(cloud_downsampled);
    double z_min = 0.01;
    double z_max = 0.3;
    extract_polygonal_prism.setHeightLimits(z_min, z_max);
    extract_polygonal_prism.segment(*segmented_cloud_inliers);

    //Find clusters of the inlier points
    pcl::search::OrganizedNeighbor<pcl::PointXYZ>::Ptr tree (new pcl::search::OrganizedNeighbor<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> clusters_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_downsampled);
    ec.setIndices (segmented_cloud_inliers);
    ec.extract (clusters_indices);

    float closest_dist = 100.0;
    Eigen::Vector4f closest_centroid(0.0, 0.0, 0.0, 0.0);
    int closest_cluster_index = 0;
    Eigen::Vector4f zero_point(0.0, 0.0, 0.0, 0.0);
    for (size_t i = 0; i < clusters_indices.size(); i++)
    {
        const pcl::PointIndices& cluster_indices = clusters_indices[i];
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_downsampled, cluster_indices, centroid);
        float dist = pcl::L2_Norm(centroid, zero_point, 3);
        if (dist < closest_dist)
        {
            closest_dist = dist;
            closest_centroid = centroid;
            closest_cluster_index = i;
        }
    }
    /* std::cout << closest_dist << std::endl; */
    /* std::cout << closest_centroid << std::endl; */

    geometry_msgs::PoseStamped pose_stamped;
    if (clusters_indices.size() > 0)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_winner(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_downsampled, clusters_indices[closest_cluster_index], *cluster_winner);
        pose_stamped = estimatePose(cluster_winner);
    }
    pose_stamped.pose.position.x = closest_centroid[0];
    pose_stamped.pose.position.y = closest_centroid[1];
    pose_stamped.pose.position.z = closest_centroid[2];
    pose_stamped.header.frame_id = input->header.frame_id;
    pose_stamped.header.stamp = ros::Time::now();
    pose_pub.publish(pose_stamped);
    ROS_INFO_STREAM("Published a pose");
    ROS_INFO_STREAM("stopping listening");
    listening = false;

    if (publish_output_pc)
    {
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_downsampled, output);
        output.header.frame_id = output_pc_frame;
        output.header.stamp = ros::Time::now();

        // Publish the cloud data
        cloud_pub.publish (output);
    }
}

void event_in_cb (const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "e_start")
    {
        ROS_INFO_STREAM("starting listening");
        listening = true;
        std_msgs::String event_out_msg;
        event_out_msg.data = "e_started";
        event_out_pub.publish (event_out_msg);
    }
    /* if (msg->data == "e_stop") */
    /* { */
    /*     ROS_INFO_STREAM("stopping listening"); */
    /*     listening = false; */
    /*     std_msgs::String event_out_msg; */
    /*     event_out_msg.data = "e_stopped"; */
    /*     event_out_pub.publish (event_out_msg); */
    /* } */
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pcl_closest_obj");
    ros::NodeHandle nh("~");

    /* ros params */
    nh.param<bool>("publish_output_pc", publish_output_pc, "true");
    nh.param<std::string>("output_pc_frame", output_pc_frame, "base_link");
    nh.param<float>("x_threshold", x_threshold, 0.05);
    nh.param<float>("z_threshold", z_threshold, 1.0);

    /* Create a ROS subscriber for the input point cloud */
    ros::Subscriber pc_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > ("input", 1, cloud_cb);
    ros::Subscriber event_in_sub = nh.subscribe<std_msgs::String> ("event_in", 1, event_in_cb);
    event_out_pub = nh.advertise<std_msgs::String> ("event_out", 1);

    if (publish_output_pc)
    {
        // Create a ROS publisher for the output point cloud
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    }

    pose_pub = nh.advertise<geometry_msgs::PoseStamped> ("output_pose", 1);

    // Spin
    ros::spin ();
}
