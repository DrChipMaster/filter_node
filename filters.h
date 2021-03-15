#ifndef FILTERS_H
#define FILTERS_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/common/common.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <vector>
#include "/home/andre/catkin_ws/devel/include/alfa_dvc/FilterSettings.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

class Filters
{
public:
    Filters();
    void do_voxelfilter();
    void do_sorfilter();
    void do_dorfilter(pcl::PointCloud<PointT>::Ptr newInput);
    void do_rorfilter();
    void do_fcsorfilter();
    void do_VoxDrorfilter();
    void do_GDRORfilter();
    void apply_filters();
    void update_filterSettings(const alfa_dvc::FilterSettings &msg);
    pcl::PointCloud<PointT>::Ptr inputCloud;
    pcl::PointCloud<PointT>::Ptr OutputCloud;

private:
    unsigned int filter_number;
    float parameter1;
    float parameter2;
    float parameter3;
    float parameter4;
    float parameter5;
    bool use_multi;

    boost::thread *m_worker_thread1;
    boost::thread *m_worker_thread2;
    boost::thread *m_worker_thread3;
    int number_threads;
    boost::mutex mutex;
    int lastFilterNumber;
    int totalTime;
    int frameInteration;
    int frameTime;
    vector<pcl::PointXYZ> inliners;
    vector< boost::thread *> thread_list;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    void run();
    void run_worker(int thread_number);
    void filter_point(pcl::PointXYZ point);
    void filter_pointGDROR(pcl::PointXYZ point);
     void cloud_cb (const  sensor_msgs::PointCloud2ConstPtr& cloud);
     void emit_frametime();
     void emit_exitpointcloud();

    ros::Publisher filter_metrics;
     ros::Publisher filter_pcloud;
     int pcl2_Header_seq;
};

#endif // FILTERS_H
