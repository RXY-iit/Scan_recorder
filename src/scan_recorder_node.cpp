#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <limits>
#include <opencv2/opencv.hpp>
#include <string>

struct MatchResult {
    size_t closestIndex;
    double winscore;
    double losescore;
};

class ScanRecorder {
public:
    ScanRecorder(ros::NodeHandle& nh, const std::string& topic, const std::string& filename);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    static std::vector<double> readCSV(const std::string& filename);
    static double calculateDifference(const std::vector<double>& a, const std::vector<double>& b);
    static MatchResult findClosestMatch(const std::vector<double>& input, const std::vector<std::vector<double>>& data);
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


MatchResult ScanRecorder::findClosestMatch(const std::vector<double>& input, const std::vector<std::vector<double>>& data)
{
    double min_diff = std::numeric_limits<double>::max();
    double diff;
    size_t closest_index = 0;
    for (size_t i = 0; i < data.size(); ++i) {
        double diff = calculateDifference(input, data[i]);
        std::cout << "diff current: " << diff << std::endl;
        if (diff < min_diff) {
            min_diff = diff;
            closest_index = i;
        }
    }
    
    MatchResult result;
    result.closestIndex = closest_index;
    result.winscore = min_diff;
    result.losescore = diff;
    return result;
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
    // ScanRecorder robot1_recorder(nh, "scan", "robot1_scan_data.csv");
    // ScanRecorder robot2_recorder(nh, "auto/scan", "robot2_scan_data.csv");
    // while (!robot1_recorder.end()) {
    //     ros::spinOnce();
    // }

    std::vector<double> robot1_data = ScanRecorder::readCSV("/home/ruan-x/workSpace_R/src/scan_recorder/src/robot1_scan_data.csv");
    std::vector<double> robot2_data = ScanRecorder::readCSV("/home/ruan-x/workSpace_R/src/scan_recorder/src/robot2_scan_data.csv");
    std::vector<std::vector<double>> all_data = {robot1_data, robot2_data};
    std::cout << "merge data ok" << std::endl;
    MatchResult result = ScanRecorder::findClosestMatch(input_data, all_data);
    std::cout << "id return:"<<result.closestIndex << std::endl;
    std::string text, text_1;
    std::string imagePath;
    // 結果出力
    if(result.closestIndex == 0){
        ROS_INFO("Robot 1 Win: Human");
        text_1 = std::to_string(result.winscore);
        text = "You Win! ";
        // Specify the path to the downloaded image
        imagePath = "/home/ruan-x/workSpace_R/src/scan_recorder/picture/robot_loose.png";
    }else{
        ROS_INFO("Robot 2 Win: Auto");
        text_1 = std::to_string(result.winscore);
        text = "Robot Win!";
        // Specify the path to the downloaded image
        imagePath = "/home/ruan-x/workSpace_R/src/scan_recorder/picture/robot_win.png";
    }

    // Define text properties for the first string
    std::string text1 = text;
    cv::Point position1(50, 250); // Position of the first text (x, y)
    cv::Scalar color1(0, 69, 225); // Text color for the first string (B, G, R)
    int fontFace1 = cv::FONT_HERSHEY_SIMPLEX; // Font type for the first string
    double fontScale1 = 3.0; // Font scale for the first string
    int thickness1 = 8; // Thickness of the first string

    // Define text properties for the second string
    std::string text2 = "Score : " + text_1;
    cv::Point position2(50, 100); // Position of the second text (x, y)
    cv::Scalar color2(255, 255, 0); // Text color for the second string (B, G, R)
    int fontFace2 = cv::FONT_HERSHEY_SIMPLEX; // Font type for the second string
    double fontScale2 = 1.5; // Font scale for the second string
    int thickness2 = 3; // Thickness of the second string

    
    // Load the image from the specified path
    cv::Mat image = cv::imread(imagePath);
    if (image.empty()) {
        std::cerr << "Failed to load image from: " << imagePath << std::endl;
        return 1;
    }
    // Generate some example image data
    // cv::Mat image(300, 400, CV_8UC3, cv::Scalar(255, 255, 255)); // White background
    // cv::rectangle(image, cv::Point(100, 100), cv::Point(300, 200), cv::Scalar(0, 0, 255), -1); // Red rectangle

    // Add text to the image
    cv::putText(image, text1, position1, fontFace1, fontScale1, color1, thickness1);

    // Add the second text to the image
    cv::putText(image, text2, position2, fontFace2, fontScale2, color2, thickness2);

    // Get the current path
    std::string  currentPath = "/home/ruan-x/workSpace_R/src/scan_recorder/picture";

    // Specify the filename for saving the image
    std::string filename = currentPath + "/Score result.png";
    cv::imwrite(filename, image);

    // Display the image using OpenCV's imshow function
    cv::imshow("Example Image", image);
    cv::waitKey(0); // Wait for a key press

    // std::cout << "spin start" << std::endl;
   while (ros::ok()) {
        //do notiong;
    }
    ros::spin();
    return 0;
}
