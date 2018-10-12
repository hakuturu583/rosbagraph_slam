//headers in rosbgraph_slam
#include <rosbagraph_slam/rosbag_reader.h>

rosbag_reader::rosbag_reader(std::string target_rosbag_path) : target_rosbag_path_(target_rosbag_path)
{
    if(!rosbag_file_exist_())
    {
        std::exit(-1);
    }
}

rosbag_reader::~rosbag_reader()
{

}

bool rosbag_reader::rosbag_file_exist_()
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

bool rosbag_reader::read(std::string nmea_topic, std::string imu_topic, std::string pointcloud_topic)
{
    rosbag::Bag bag;
    bag.open(target_rosbag_path_, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(nmea_topic);
    topics.push_back(imu_topic);
    topics.push_back(pointcloud_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    foreach(rosbag::MessageInstance const m, view)
    {

    }
}