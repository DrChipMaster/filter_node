 #include "ros/ros.h"
#include "filters.h"
#include "rosthread.h"

int main(int argc, char **argv)
{
    pcl::PointCloud<PointT>::Ptr inputCloud;
    pcl::PointCloud<PointT>::Ptr OutputCloud;
     inputCloud.reset (new PointCloudT);
     OutputCloud.reset (new PointCloudT);
     ros::init (argc, argv, "alfa_node");
       if (!ros::master::check()) {
           cout <<"Failed to inicialize ros"<<endl;
       }
    Filters mFilter;
    mFilter.inputCloud = inputCloud;
    mFilter.OutputCloud = OutputCloud;
    RosThread mRosThread(&mFilter,inputCloud);
    while(ros::ok())
    {

    }
}
