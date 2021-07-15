#include "rosthread.h"
#include <thread>



RosThread::RosThread(Filters *mFilters, pcl::PointCloud<PointT>::Ptr pcloud)
{
    this->mFilters = mFilters;
    this->pcloud = pcloud;
    subscrive_topics();
}

void RosThread::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    //cout<<"Recieved pointcloud"<<endl;
    if ((cloud->width * cloud->height) == 0)
    {
        cout <<"Recieved empty point cloud"<<endl;
        return;
    }
    cout<<"Recieved cloud"<<endl;
    pcl::fromROSMsg(*cloud,*pcloud);
    mFilters->apply_filters();

}

void RosThread::parameters_cb(const alfa_dvc::FilterSettings &msg)
{
    //cout<<"Recieved FilterSettings... Updating"<<endl;
    mFilters->update_filterSettings(msg);

}

void RosThread::init()
{
        char arg0[]= "filter_node";

        //strcpy(aux,output.toStdString().c_str());
        char *argv[]={arg0,NULL};
        int argc=(int)(sizeof(argv) / sizeof(char*)) - 1;;
        ros::init (argc, argv, "alfa_node");
          if (!ros::master::check()) {
              cout <<"Failed to inicialize ros"<<endl;
            return;
          }

}

void RosThread::subscrive_topics()
{
    sub_cloud = nh.subscribe("alfa_pointcloud",0,&RosThread::cloud_cb,this);
    sub_parameters = nh.subscribe("alfa_filter_settings",0,&RosThread::parameters_cb,this);

    m_spin_thread = new boost::thread(&RosThread::spin, this);


}

void RosThread::spin()
{
    int threads = std::thread::hardware_concurrency();
    std::cout << "Started Spinning with processor_count threads"<<threads << std::endl;
    //ros::MultiThreadedSpinner spinner(threads);
    //spinner.spin();
    ros::spin();
}
