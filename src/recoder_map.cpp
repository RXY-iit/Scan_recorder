#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <fstream>
#include <sstream>
#include <ros/console.h>

//・・・・・・・・・・・・・・・・・・・・・・・・・・・
//////仮のマップデータ生成用、本番は使わない
//・・・・・・・・・・・・・・・・・・・・・・・・・・・
class ScanRecorder {
public:
    ScanRecorder(ros::NodeHandle& nh, const std::string& topic, const std::string& filename);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

private:
    std::string filename_;
    ros::Subscriber scan_sub_;
    std::ofstream file_;
    ros::Time start_time =  ros::Time::now();

    std::string getFilePath(const std::string& filename);
};

ScanRecorder::ScanRecorder(ros::NodeHandle& nh, const std::string& topic, const std::string& filename)
    : filename_(filename)
{
    std::string file_path = getFilePath(filename_);
    scan_sub_ = nh.subscribe(topic, 1, &ScanRecorder::scanCallback, this);
    file_.open(file_path, std::ios::out | std::ios::app);
    if (!file_.is_open()) {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
    } else {
        ROS_INFO("File opened successfully: %s", file_path.c_str());
    }
    start_time =  ros::Time::now();
}

void ScanRecorder::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    double front_sum = 0.0;
    int count = 0;
    int front_index_start = (scan->ranges.size() / 2) - 2; // Start from the middle - 2
    int front_index_end = (scan->ranges.size() / 2) + 2;   // End at the middle + 2
    for (int i = front_index_start; i <= front_index_end; ++i) {
        front_sum += scan->ranges[i];
        ++count;
    }
    double average = front_sum / count;
    // double current_time_sec = ros::Time::now().toSec();
    double current_time_sec = (ros::Time::now() - start_time).toSec();
    file_ << current_time_sec << "," << average << std::endl;
}

std::string ScanRecorder::getFilePath(const std::string& filename)
{
    std::string file_path(__FILE__);
    size_t pos = file_path.find_last_of("/");
    if (pos != std::string::npos) {
        return file_path.substr(0, pos + 1) + filename;
    } else {
        ROS_WARN("Unable to determine file path. Saving to current directory.");
        return filename;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "recorder_map_node");
    ros::NodeHandle nh;
    std::string filename = "scan_data_2.csv";
    
    ScanRecorder recorder(nh, "/scan", filename);

    ros::spin();
    return 0;
}
