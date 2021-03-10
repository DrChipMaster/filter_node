#ifndef ROSTHREAD_H
#define ROSTHREAD_H

#include <ros/ros.h>


// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "filters.h"
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>



using namespace std;
class RosThread
{
public:
    RosThread(Filters* mFilters, pcl::PointCloud<PointT>::Ptr pcloud);

private:
    Filters* mFilters;
    void cloud_cb (const  sensor_msgs::PointCloud2ConstPtr& cloud);
    void parameters_cb(const alfa_dvc::FilterSettings &msg);
    ros::Subscriber sub_cloud;
    ros::Subscriber sub_parameters;
    ros::NodeHandle nh;
    pcl::PointCloud<PointT>::Ptr pcloud;
    void init();
    void subscrive_topics();
    boost::thread *m_spin_thread;
    void spin();



};

#endif // ROSTHREAD_H
