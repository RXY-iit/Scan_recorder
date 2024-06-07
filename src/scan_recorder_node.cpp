#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <limits>

class ScanRecorder {
public:
    ScanRecorder(ros::NodeHandle& nh, const std::string& topic, const std::string& filename);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    static std::vector<double> readCSV(const std::string& filename);
    static double calculateDifference(const std::vector<double>& a, const std::vector<double>& b);
    static double findClosestMatch(const std::vector<double>& input, const std::vector<std::vector<double>>& data);
    bool end();

private:
    std::string filename_;
    ros::Subscriber scan_sub_;
    std::ofstream file_;
    ros::Time start_time =  ros::Time::now();

    std::string getFilePath(const std::string& filename);
    bool end_;
};

ScanRecorder::ScanRecorder(ros::NodeHandle& nh, const std::string& topic, const std::string& filename)
    : filename_(filename)
{
    std::string file_path = getFilePath(filename_);
    scan_sub_ = nh.subscribe(topic, 1, &ScanRecorder::scanCallback, this);
    file_.open(file_path, std::ios::out | std::ios::app);
    if (!file_.is_open()) {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
    }
    start_time =  ros::Time::now();
    end_ = false;
}

bool ScanRecorder::end() { return end_; }

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
    double current_time_sec = (ros::Time::now() - start_time).toSec(); // Set current time to 0
    file_ << current_time_sec << "," << average << std::endl;
    std::cout << current_time_sec << "," << average << std::endl;
    //１０秒まで記録。本番お時はマップ動画の時間より設定
    if(current_time_sec>=10){
        end_ = true;
    }
}

std::vector<double> ScanRecorder::readCSV(const std::string& filename)
{
    std::ifstream file(filename);
    std::vector<double> data;
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream line_stream(line);
        std::string cell;
        int column_index = 0;

        while (std::getline(line_stream, cell, ',')) {
            column_index++;
            if (column_index == 2) { // Only save data from the second column
                data.push_back(std::stod(cell));
                
            }
        }
    }
    return data;
}

double ScanRecorder::calculateDifference(const std::vector<double>& a, const std::vector<double>& b)
{
    size_t min_size = std::min(a.size(), b.size());
    // size_t min_size = a.size();
    double sum = 0.0;
    size_t count = 0;

    for (size_t j = 0; j < min_size; ++j) {
        sum += std::abs(a[j] - b[j]);
        ++count;
    }

    if (count == 0) {
        return 0.0; // Return 0 if no valid elements found in b
    }

    return sum;
}


double ScanRecorder::findClosestMatch(const std::vector<double>& input, const std::vector<std::vector<double>>& data)
{
    double min_diff = std::numeric_limits<double>::max();
    size_t closest_index = 0;
    for (size_t i = 0; i < data.size(); ++i) {
        double diff = calculateDifference(input, data[i]);
        std::cout << "diff current: " << diff << std::endl;
        if (diff < min_diff) {
            min_diff = diff;
            closest_index = i;
        }
    }

    // ROS_INFO("Closest match found at index: %zu with average difference: %f", closest_index, min_diff);
    return closest_index;
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
    ros::init(argc, argv, "scan_recorder_node");
    ros::NodeHandle nh;
    ros::Rate rate(10.0);
    std::string map_center_csv = "/home/ruan-x/workSpace_R/src/scan_recorder/csv/map_center.csv";
    std::vector<double> input_data = ScanRecorder::readCSV(map_center_csv);
    std::cout << "scanrecorder start" << std::endl;
    ScanRecorder robot1_recorder(nh, "scan", "robot1_scan_data.csv");
    // ScanRecorder robot2_recorder(nh, "/robot2/scan", "robot2_scan_data.csv");
    while (!robot1_recorder.end()) {
        ros::spinOnce();
    }

    std::vector<double> robot1_data = ScanRecorder::readCSV("/home/ruan-x/workSpace_R/src/scan_recorder/src/robot1_scan_data.csv");
    std::vector<double> robot2_data = ScanRecorder::readCSV("/home/ruan-x/workSpace_R/src/scan_recorder/csv/robot2_scan_data.csv");
    std::vector<std::vector<double>> all_data = {robot1_data, robot2_data};
    std::cout << "merge data ok" << std::endl;
    double Win_ID = ScanRecorder::findClosestMatch(input_data, all_data);
    std::cout << "id return:"<<Win_ID << std::endl;
    // 結果出力
    if(Win_ID == 0){
        ROS_INFO("Robot 1 Win: Human");
    }else{
        ROS_INFO("Robot 2 Win: Auto");
    }

    // std::cout << "spin start" << std::endl;
   while (ros::ok()) {
        //do notiong;
    }
    ros::spin();
    return 0;
}
