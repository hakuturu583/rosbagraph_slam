//headers in rosbgraph_slam
#include <rosbagraph_slam/rosbag_reader.h>

RosbagReader::RosbagReader(std::string target_rosbag_path) : target_rosbag_path_(target_rosbag_path)
{
    if(!rosbag_file_exist_())
    {
        std::exit(-1);
    }
}

RosbagReader::~RosbagReader()
{

}

bool RosbagReader::rosbag_file_exist_()
{
    namespace fs = boost::filesystem;
    fs::path path(target_rosbag_path_);
    boost::system::error_code error;
    const bool result = fs::exists(path, error);
    if (!result || error)
    {
        ROS_ERROR_STREAM("rosbag path is invalid.");
        return false;
    }
    return true;
}

std::vector<sensor_msgs::PointCloud2::ConstPtr> RosbagReader::read(std::string pointcloud_topic)
{
    std::vector<sensor_msgs::PointCloud2::ConstPtr> ret;
    rosbag::Bag bag;
    bag.open(target_rosbag_path_, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(pointcloud_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::PointCloud2::ConstPtr pc =  m.instantiate<sensor_msgs::PointCloud2>();
        ret.push_back(pc);
    }
    return ret;
}