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

typedef long long int u64;
typedef pcl::PointXYZI PointT;
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
    void do_LIORfilter();
    void do_DLIORfilter();
    void apply_filters();
    void update_filterSettings(const alfa_dvc::FilterSettings &msg);
    void do_hardwarefilter();


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
    bool hardware_ready;

    boost::thread *m_worker_thread1;
    boost::thread *m_worker_thread2;
    boost::thread *m_worker_thread3;
    int number_threads;
    boost::mutex mutex;
    int lastFilterNumber;
    int totalTime;
    int frameInteration;
    int frameTime;
    vector<pcl::PointXYZI> inliners;
    vector< boost::thread *> thread_list;

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    void run();
    void run_worker(int thread_number);
    void run_lior_worker(int thread_number);
    void run_dlior_worker(int thread_number);
    void filter_point(pcl::PointXYZI point,bool isDLIOR=0);
    void filter_pointGDROR(pcl::PointXYZI point);
    void filter_pointROR(pcl::PointXYZI point);
     void cloud_cb (const  sensor_msgs::PointCloud2ConstPtr& cloud);
     void decode_pointcloud();

     void emit_frametime();
     void emit_exitpointcloud();

    ros::Publisher filter_metrics;
     ros::Publisher filter_pcloud;
     int pcl2_Header_seq;



     //Hardware parameters:
     unsigned int bram_size = 0x40000;
     off_t bram_x = 0xA0000000; // physical base address
     off_t bram_y =0xA1000000;  // physical base address
     off_t bram_z =0xA2000000; // physical base address
     u64 *bram_x_ptr;
     u64 *bram_y_ptr;
     u64 *bram_z_ptr;
};

#endif // FILTERS_H
