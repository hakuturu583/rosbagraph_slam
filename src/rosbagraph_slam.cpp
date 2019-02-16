//headers in rosbgraph_slam
#include <rosbagraph_slam/rosbagraph_slam.h>

RosbagraphSlam::RosbagraphSlam(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("rosbag_filepath", rosbag_filepath_, "");
    pnh_.param<std::string>("pointcloud_topic", pointcloud_topic_, "/points_raw");
    reader_ptr_ = boost::make_shared<RosbagReader>(rosbag_filepath_);
    std::vector<sensor_msgs::PointCloud2::ConstPtr> point_clouds = reader_ptr_->read(pointcloud_topic_);
    ROS_INFO_STREAM("read rosbag file finished, " << point_clouds.size() << " point clouds were found.");
}

RosbagraphSlam::~RosbagraphSlam()
{
    
}