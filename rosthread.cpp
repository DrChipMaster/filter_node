#include "rosthread.h"



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
    sub_cloud = nh.subscribe("alfa_pointcloud",2,&RosThread::cloud_cb,this);
    sub_parameters = nh.subscribe("alfa_filter_settings",2,&RosThread::parameters_cb,this);

    m_spin_thread = new boost::thread(&RosThread::spin, this);


}

void RosThread::spin()
{
    std::cout << "Started Spinning." << std::endl;
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
}
