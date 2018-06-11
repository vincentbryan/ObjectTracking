//
// Created by vincent on 18-5-30.
//

#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>


#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <cmath>
#include <pcl_ros/impl/transforms.hpp>

#include <Grids.h>
#include <Segmentation.h>
#include <Classification.h>

ros::Publisher gGroundPublisher;
ros::Publisher gObstalcePublisher;

void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    static int frame_id = 0;
    ROS_INFO_STREAM("Frame : " << frame_id++);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_raw_data(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_processed_data(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_ground_data(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_evaluated_data(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_clustered_data(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::fromROSMsg(*input, *ptr_raw_data);

    //region Preprocess
    double radius_min = 4.0, radius_max = 18.0;

    for(auto it = ptr_raw_data->begin(); it != ptr_raw_data->end(); ++it)
    {
        double distance = std::sqrt(it->x * it->x + it->y * it->y);
        if (distance > radius_min && distance < radius_max)
            ptr_processed_data->push_back(*it);
    }
    //endregion

    Grids * grids = new Grids(ptr_processed_data, 0.5f);
    Segmentation segmentation(grids, ptr_processed_data);

    ptr_ground_data = segmentation.GetGroundData();
    ptr_evaluated_data = segmentation.GetEvaluateData();

    Classification classification(grids, ptr_evaluated_data);
    ptr_clustered_data = classification.GetColoredCloud();

    //region output

    ptr_ground_data->header.frame_id = ptr_raw_data->header.frame_id;
    ptr_evaluated_data->header.frame_id = ptr_raw_data->header.frame_id;
    ptr_clustered_data->header.frame_id = ptr_raw_data->header.frame_id;

    sensor_msgs::PointCloud2 ground_msg;
    sensor_msgs::PointCloud2 obstacle_msg;

    pcl::toROSMsg(*ptr_ground_data, ground_msg);
    pcl::toROSMsg(*ptr_clustered_data, obstacle_msg);

    gGroundPublisher.publish(ground_msg);
    gObstalcePublisher.publish(obstacle_msg);

    //endregion
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "object_tracking");
    ros::NodeHandle nodeHandle;
    tf::TransformListener transform_listener(ros::Duration(100));

    ros::Subscriber subscriber = nodeHandle.subscribe("velodyne_points", 160, callback);
    gGroundPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("ground_cloud", 1);
    gObstalcePublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("obstacle_cloud", 1);

    auto test= std::function<void(int)>();
    test = [&](int i)
    {
        if(i > 0)
        {
            std::cout << i << std::endl;
            test(--i);
        }
        else
        {
            return;
        }

    };
    test(10);

    ros::spin();

    return 0;
}