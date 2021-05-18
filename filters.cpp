#include "filters.h"
#include <random>
#include<time.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Int32.h>
#include <chrono>



#define LIOR_CONST  0.066





Filters::Filters()
{
    pcl2_Header_seq=0;
    filter_number = 1;
    parameter1 = 0.1;
    parameter2 = 0.1;
    parameter3 = 0.1;
    parameter4 = 0.1;
    parameter5 = 0.1;
    use_multi  = false;

}

void Filters::do_voxelfilter()
{
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (inputCloud);
    sor.setLeafSize (parameter1,parameter2,parameter3);
    sor.filter (*OutputCloud);

}

void Filters::do_sorfilter()
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud (inputCloud);
    sor.setMeanK (parameter1);
    sor.setStddevMulThresh (parameter2);
    sor.filter (*OutputCloud);
}

void Filters::do_dorfilter(pcl::PointCloud<PointT>::Ptr newInput)
{
    vector<pcl::PointXYZI> outliners;

    inliners.clear();
    kdtree.setInputCloud(newInput);

    if(use_multi == true)
    {
        thread_list.clear();
        cout << "Running multithreading with "<< number_threads<<endl;
        for (int i =0;i <= number_threads;i++)
        {
            thread_list.push_back(new boost::thread(&Filters::run_worker, this,i));
        }
        for (int i =0;i <= number_threads;i++)
        {
            thread_list[i]->join();
        }

    }
    else {
        for (auto &point : *newInput)
        {
            filter_point(point);
        }
    }
    OutputCloud->resize(inliners.size());
    int counter =0;
    for(auto& point: *OutputCloud)
    {
        point = inliners[counter];
        counter++;
    }
}

void Filters::do_rorfilter()
{
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
    outrem.setInputCloud(inputCloud);
    outrem.setRadiusSearch(parameter1);
    outrem.setMinNeighborsInRadius (parameter2);
    outrem.filter (*OutputCloud);

}

void Filters::do_fcsorfilter()
{
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D (*inputCloud, minPt, maxPt);
    double NumberClusters = parameter1;
    double  ClusterLength = abs(maxPt.x -minPt.x)/NumberClusters;
    double ClusterWidth = abs(maxPt.y - minPt.y)/NumberClusters;
    double ClusterHeight = abs(maxPt.z - minPt.z)/NumberClusters;
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud (inputCloud);
    voxel.setLeafSize (ClusterLength,ClusterWidth, ClusterHeight);
    pcl::PointCloud<PointT>::Ptr betweenCloud( new pcl::PointCloud<pcl::PointXYZI>);
    voxel.filter (*betweenCloud);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud (betweenCloud);
    sor.setMeanK (parameter2);
    sor.setStddevMulThresh (parameter3);
    sor.filter (*OutputCloud);
}

void Filters::do_VoxDrorfilter()
{
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D (*inputCloud, minPt, maxPt);
    double NumberClusters = parameter5;
    double  ClusterLength = abs(maxPt.x -minPt.x)/NumberClusters;
    double ClusterWidth = abs(maxPt.y - minPt.y)/NumberClusters;
    double ClusterHeight = abs(maxPt.z - minPt.z)/NumberClusters;
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud (inputCloud);
    voxel.setLeafSize (ClusterLength,ClusterWidth, ClusterHeight);
    pcl::PointCloud<PointT>::Ptr betweenCloud( new pcl::PointCloud<pcl::PointXYZI>);
    voxel.filter (*betweenCloud);
    do_dorfilter(betweenCloud);

}

void Filters::do_GDRORfilter()
{
    //vector<pcl::PointXYZI> inliners;
    inliners.clear();
    kdtree.setInputCloud(inputCloud);

    if(use_multi)
    {
        thread_list.clear();
        cout << "Running multithreading with "<< number_threads<<endl;
        for (int i =0;i <= number_threads;i++)
        {
            thread_list.push_back(new boost::thread(&Filters::run_worker, this,i));
        }
        for (int i =0;i <= number_threads;i++)
        {
            thread_list[i]->join();
        }
    }
    else
    {
        for(auto& point: *inputCloud)
        {
            if(point.z >0- parameter5)
            {
                filter_point(point);
            }

        }
    }

    OutputCloud->resize(inliners.size());
    int counter =0;
    for(auto& point: *OutputCloud)
    {
        point = inliners[counter];
        counter++;
    }

}


void Filters::do_LIORfilter()
{
    inliners.clear();
    kdtree.setInputCloud(inputCloud);
    number_threads = parameter4;
    if (use_multi && number_threads >1)
    {
        thread_list.clear();
        for (int i =0;i <= number_threads;i++)
        {
            thread_list.push_back(new boost::thread(&Filters::run_lior_worker, this,i));
        }
        for (int i =0;i <= number_threads;i++)
        {
            thread_list[i]->join();
         }
    }
    else {
        for (auto &point : *inputCloud)
        {
            double intensity_trehshold=parameter4;
            if(point._PointXYZI::intensity > intensity_trehshold)
            {
                mutex.lock();
                inliners.push_back(point);
                mutex.unlock();
            }
            else
            {
                filter_pointROR(point);
            }
        }

    }

    OutputCloud->resize(inliners.size());
    int counter = 0;
    for (auto &point : *OutputCloud)
    {
        point = inliners[counter];
        counter++;
    }

}

void Filters::do_DLIORfilter()
{
    inliners.clear();
    kdtree.setInputCloud(inputCloud);
    number_threads = parameter5;
    if (use_multi && number_threads >1)
    {
        thread_list.clear();
        for (int i =0;i <= number_threads;i++)
        {
            thread_list.push_back(new boost::thread(&Filters::run_dlior_worker, this,i));
        }
        for (int i =0;i <= number_threads;i++)
        {
            thread_list[i]->join();
         }
    }
    else {
        for (auto &point : *inputCloud)
        {
            double intensity_trehshold=parameter4;
            if(point._PointXYZI::intensity > intensity_trehshold)
            {
                mutex.lock();
                inliners.push_back(point);
                mutex.unlock();
            }
            else
            {
                filter_point(point);
            }
        }

    }

    OutputCloud->resize(inliners.size());
    int counter = 0;
    for (auto &point : *OutputCloud)
    {
        point = inliners[counter];
        counter++;
    }

}

using namespace std::chrono;
void Filters::apply_filters()
{
    auto start = high_resolution_clock::now();

    switch (filter_number) {
    case 1:
        do_voxelfilter();
        break;
    case 2:
        do_rorfilter();
        break;
    case 3:
        do_sorfilter();
        break;
    case 4:
        do_dorfilter(inputCloud);
        break;
    case 5:
        do_fcsorfilter();
        break;
    case 6:
        do_VoxDrorfilter();
        break;
    case 7:
        do_GDRORfilter();
        break;
    case 8:
        do_LIORfilter();
        break;
    case 9:
        do_DLIORfilter();
        break;
    }
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    frameTime = duration.count();
    emit_frametime();
    emit_exitpointcloud();

}



void Filters::filter_pointROR(pcl::PointXYZI point)
{
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    int neighbors = kdtree.radiusSearch(point,parameter1, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    if (neighbors >parameter3)
    {
        inliners.push_back(point);
    }
}



void Filters::filter_point(pcl::PointXYZI point)
{
    float distance = sqrt(pow(point.x,2)+pow(point.y,2));
    float search_radius;
    //float anlge = atan2((double)point.y,(double)point.x);
    //if(anlge <0)
    //{
    //anlge = 2*M_PI + anlge;
    //}
    float anlge = parameter4;

    if(distance<parameter1)
    {
        search_radius = parameter1;
    }else
    {
        //float anlge = atan((double)point.y/(double)point.x);


        //anlge = anlge * 180.0 / M_PI;

        search_radius = parameter2 * (distance*anlge);
    }
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    int neighbors = kdtree.radiusSearch (point, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) ;
    if(neighbors>=parameter3)
    {
        mutex.lock();

        inliners.push_back(point);
        mutex.unlock();
        //OutputCloud->push_back(point);

        //}
        //}
    }
}

void Filters::filter_pointGDROR(pcl::PointXYZI point)
{
    if(point.z > parameter5)
    {


        float distance = sqrt(pow(point.x,2)+pow(point.y,2));
        float search_radius;
        //float anlge = atan2((double)point.y,(double)point.x);
        //if(anlge <0)
        //{
        //anlge = 2*M_PI + anlge;
        //}
        float anlge = parameter4;

        if(distance<parameter1)
        {
            search_radius = parameter1;
        }else
        {
            //float anlge = atan((double)point.y/(double)point.x);


            //anlge = anlge * 180.0 / M_PI;

            search_radius = parameter2 * (distance*anlge);
        }
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        int neighbors = kdtree.radiusSearch (point, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) ;
        if(neighbors>=parameter3)
        {
            mutex.lock();

            inliners.push_back(point);
            mutex.unlock();
            //OutputCloud->push_back(point);

            //}
            //}
        }
    }
}

void Filters::update_filterSettings(const alfa_dvc::FilterSettings &msg)
{
    cout<<"updating filter Settings"<<endl;
    mutex.lock();
    filter_number = msg.filterNumber;
    parameter1 = msg.parameter1;
    parameter2 = msg.parameter2;
    parameter3 = msg.parameter3;
    parameter4 = msg.parameter4;
    parameter5 = msg.parameter5;

    switch (filter_number) {
    case 1:
        cout << "Updated to voxel Filter";
        break;
    case 2:
        cout << "Updated to ROR Filter";
        break;
    case 3:
        cout << "Updated to SOR Filter";
        break;
    case 4:
        cout << "Updated to DROR Filter";
        break;
    case 5:
        cout << "Updated to FCSOR Filter";
        break;
    case 6:
        cout << "Updated to VDROR Filter";
        break;
    case 7:
        cout << "Updated to GDROR Filter";
        break;
    case 8:
        cout << "Updated to LIOR Filter";
        break;
    case 9:
        cout << "Updated to DLIOR Filter";
        break;
    }
    cout<< " with parameters: 1:"<<msg.parameter1<<"; 2:"<<msg.parameter2<<"; 3:"<<msg.parameter3<<"; 4:"<<msg.parameter4<<"; 5:"<<msg.parameter5<<endl;
    mutex.unlock();
    if (thread_list.size()>0)
    {
        for (int i =0;i <= number_threads;i++)
        {
            thread_list[i]->join();
        }
    }

    //thread_list.clear();
    mutex.lock();
    if(filter_number == 4)
    {
        if (parameter5 > 0)
        {
            cout << "Enabling Multihreading"<<endl;
            use_multi = true;
            number_threads = int(parameter5);
        }
        else
        {
            cout << "Disabling Multihreading"<<endl;
            use_multi = false;
        }
    }
        mutex.unlock();

}

void Filters::run_worker(int thread_number)
{

    for(int i =(inputCloud->size()/number_threads)*thread_number; i<= (inputCloud->size()/number_threads)*(thread_number+1);i++)
    {
        pcl::PointXYZI point = (*inputCloud)[i];
        if (filter_number == 4)
        filter_point(point);
        else filter_pointGDROR(point);

    }
}




void Filters::run_lior_worker(int thread_number)
{
    for(int i =(inputCloud->size()/number_threads)*thread_number; i<= (inputCloud->size()/number_threads)*(thread_number+1);i++)
    {
        pcl::PointXYZI point = (*inputCloud)[i];
        double intensity_trehshold=parameter2;
        float distance = sqrt(pow(point.x, 2) + pow(point.y, 2)+pow(point.z, 2));
        if(point._PointXYZI::intensity*distance*LIOR_CONST > intensity_trehshold)
        {
            mutex.lock();
            inliners.push_back(point);
            mutex.unlock();
        }
        else
        {
            filter_pointROR(point);
        }
    }
}



void Filters::run_dlior_worker(int thread_number)
{
    for(int i =(inputCloud->size()/number_threads)*thread_number; i<= (inputCloud->size()/number_threads)*(thread_number+1);i++)
    {
        pcl::PointXYZI point = (*inputCloud)[i];
        double intensity_trehshold=parameter4;
        float distance = sqrt(pow(point.x, 2) + pow(point.y, 2)+pow(point.z, 2));
        if(point._PointXYZI::intensity*distance*LIOR_CONST > intensity_trehshold)
        {
            mutex.lock();
            inliners.push_back(point);
            mutex.unlock();
        }
        else
        {
            filter_point(point);
        }
    }

}


void Filters::emit_frametime()
{
    ros::NodeHandle n;
    std_msgs::Int32 msg;
    filter_metrics = n.advertise<std_msgs::Int32>("alfa_filter_metrics", 1);
    msg.data = frameTime;
    filter_metrics.publish(msg);

}

void Filters::emit_exitpointcloud()
{
    ros::NodeHandle n;
    filter_pcloud = n.advertise<sensor_msgs::PointCloud2>("alfa_output_pcloud", 2);
    sensor_msgs::PointCloud2 pcl2_frame;
    pcl::toROSMsg(*OutputCloud,pcl2_frame);
    pcl2_frame.header.frame_id = "PclNoise";
    pcl2_frame.header.seq = pcl2_Header_seq;
    pcl2_frame.header.stamp = ros::Time::now();
    pcl2_Header_seq++;
    filter_pcloud.publish(pcl2_frame);
    //cout<<"emitted pointcloud";
}
