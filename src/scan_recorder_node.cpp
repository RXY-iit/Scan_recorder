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
    void record();

private:
    std::string filename_;
    ros::Subscriber scan_sub_;
    std::ofstream file_;
    ros::Time start_time =  ros::Time::now();
    sensor_msgs::LaserScan::ConstPtr latest_scan_; // Store the latest scan message

    std::string getFilePath(const std::string& filename);
    bool end_;
};

ScanRecorder::ScanRecorder(ros::NodeHandle& nh, const std::string& topic, const std::string& filename)
    : filename_(filename)
{
    std::string file_path = getFilePath(filename_);
    scan_sub_ = nh.subscribe(topic, 1, &ScanRecorder::scanCallback, this);
    // file_.open(file_path, std::ios::out | std::ios::app);
    file_.open(file_path, std::ios::out);
    if (!file_.is_open()) {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
    }
    start_time =  ros::Time::now();
    end_ = false;
}

bool ScanRecorder::end() { return end_; }

void ScanRecorder::record() {
    if (latest_scan_) {
        double front_sum = 0.0;
        int count = 0;
        int front_index_start = (latest_scan_->ranges.size() / 2) - 2; // Start from the middle - 2
        int front_index_end = (latest_scan_->ranges.size() / 2) + 2;   // End at the middle + 2
        for (int i = front_index_start; i <= front_index_end; ++i) {
            front_sum += latest_scan_->ranges[i];
            ++count;
        }
        double average = front_sum / count;
        double current_time_sec = (ros::Time::now() - start_time).toSec(); // Set current time to 0
        file_ << current_time_sec << "," << average << std::endl;
        // std::cout << current_time_sec << "," << average << std::endl;
        // Record for up to 60 seconds. Adjust as needed for the actual duration.
        if (current_time_sec >= 22) {
            end_ = true;
        }
    }
}

void ScanRecorder::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
   latest_scan_ = scan; // Update the latest scan data
   latest_scan_ = scan; // Update the latest scan data
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
    double max_diff = std::numeric_limits<double>::min();
    double diff;
    size_t closest_index = 0;
    for (size_t i = 0; i < data.size(); ++i) {
        double diff = calculateDifference(input, data[i]);
        std::cout << "diff current: " << diff << std::endl;
        if (diff < min_diff) {
            min_diff = diff;
            closest_index = i;
        }
        if (diff > max_diff) {
            max_diff = diff;
            // closest_index = i;
        }
    }

    // Map min_diff to a score between 0 and 100
    double winscore = 100.0 - ((min_diff-150) / 200) * 100.0;
    double losescore = 100.0 - ((max_diff-150) / 200) * 100.0;
    
    MatchResult result;
    result.closestIndex = closest_index;
    result.winscore = winscore;
    result.losescore = losescore;
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

    // Open video file
    cv::VideoCapture cap("/home/ruan-x/workSpace_R/src/scan_recorder/video/hogehoge.mp4");
    if (!cap.isOpened()) {
        std::cerr << "Error opening video file" << std::endl;
        return -1;
    }

    // // Get the frame rate of the video
    // double fps = cap.get(cv::CAP_PROP_FPS);
    // if (fps <= 0) {
    //     std::cerr << "Error retrieving frame rate" << std::endl;
    //     return -1;
    // }

    // // Calculate delay in milliseconds
    // int delay = static_cast<int>(1000 / fps);
    ScanRecorder robot1_recorder(nh, "scan", "robot1_scan_data.csv");
    ScanRecorder robot2_recorder(nh, "auto/scan", "robot2_scan_data.csv");
    cv::Mat frame;

//    while (ros::ok() && !robot1_recorder.end() && !robot2_recorder.end()) {
    while (ros::ok() ) {
        ros::spinOnce();

        // Read frame from video
        cap >> frame;
        if (frame.empty()) {
            std::cout << "End of video stream" << std::endl;
            break;
        }

        cv::namedWindow("Video", cv::WINDOW_NORMAL);
        cv::moveWindow("Video", 0, 0);
        // Display video frame
        cv::imshow("Video", frame);
        if (cv::waitKey(30) == 'q') {
            std::cout << "User requested to quit." << std::endl;
            break;
        }

        // Record scan data
        if (!robot1_recorder.end()) {
            robot1_recorder.record();
        }
        if (!robot2_recorder.end()) {
            robot2_recorder.record();
        }

        // rate.sleep();
    }
    // while (!robot1_recorder.end()) {
    //     ros::spinOnce();
    // }

    std::vector<double> robot1_data = ScanRecorder::readCSV("/home/ruan-x/workSpace_R/src/scan_recorder/src/robot1_scan_data.csv");
    std::vector<double> robot2_data = ScanRecorder::readCSV("/home/ruan-x/workSpace_R/src/scan_recorder/src/robot2_scan_data.csv");
    std::vector<std::vector<double>> all_data = {robot1_data, robot2_data};
    std::cout << "merge data ok" << std::endl;
    MatchResult result = ScanRecorder::findClosestMatch(input_data, all_data);
    std::cout << "id return:"<<result.closestIndex << std::endl;
    std::string text, text_1,text_2;
    std::string imagePath;
    std::stringstream stream,stream1;
    
    // // 結果出力
    if(result.closestIndex == 0){
        ROS_INFO("Robot 1 Win: Human");
        // text_1 = std::to_string(result.winscore);
        stream << std::fixed << std::setprecision(2) << result.winscore;
        text_1 = "Your Score : " + stream.str();
        stream1 << std::fixed << std::setprecision(2) << result.losescore;
        text_2 = "Robot Score: " + stream1.str();
        text = "Congratulations!!!! ";

        // Specify the path to the downloaded image
        imagePath = "/home/ruan-x/workSpace_R/src/scan_recorder/picture/robot_loose.png";
    }else{
        ROS_INFO("Robot 2 Win: Auto");
        stream << std::fixed << std::setprecision(2) << result.winscore;
        text_1 = "Robot: " + stream.str();
        stream1 << std::fixed << std::setprecision(2) << result.losescore;
        text_2 = "You: " + stream1.str();
        // text_1 = std::to_string(result.winscore);
        text = "Sorry, You Fail!!!";
        // Specify the path to the downloaded image
        imagePath = "/home/ruan-x/workSpace_R/src/scan_recorder/picture/robot_win.png";

    }

    // Define text properties for the first string
    std::string text1 = text;
    cv::Point position1(50, 450); // Position of the first text (x, y)
    cv::Scalar color1(0, 69, 225); // Text color for the first string (B, G, R)
    int fontFace1 = cv::FONT_HERSHEY_COMPLEX_SMALL; // Font type for the first string
    double fontScale1 = 3.5; // Font scale for the first string
    int thickness1 = 10; // Thickness of the first string

    // Define text properties for the second string
    std::string text2 = text_1;
    cv::Point position2(500, 550); // Position of the second text (x, y)
    cv::Scalar color2(128, 223, 225); // Text color for the second string (B, G, R)
    int fontFace2 = cv::FONT_HERSHEY_COMPLEX_SMALL; // Font type for the second string
    double fontScale2 = 4.0; // Font scale for the second string
    int thickness2 = 8; // Thickness of the second string

    // Define text properties for the second string
    std::string text3 =  text_2;
    cv::Point position3(500,710); // Position of the second text (x, y)
    cv::Scalar color3(128, 223, 225); // Text color for the second string (B, G, R)
    int fontFace3 = cv::FONT_HERSHEY_COMPLEX_SMALL; // Font type for the second string
    double fontScale3 = 4; // Font scale for the second string
    int thickness3 = 8; // Thickness of the second string

    
    // Load the image from the specified path
    cv::Mat image = cv::imread(imagePath);
    if (image.empty()) {
        std::cerr << "Failed to load image from: " << imagePath << std::endl;
        return 1;
    }
    // Generate some example image data
    // cv::Mat image(300, 400, CV_8UC3, cv::Scalar(255, 255, 255)); // White background
    // cv::rectangle(image, cv::Point(100, 100), cv::Point(300, 200), cv::Scalar(0, 0, 255), -1); // Red rectangle
    // Define screen resolution
     // Resize the image to fit the screen size
    cv::Size screenSize(1920, 1080);
    cv::resize(image, image, screenSize);
    
    // Create a blank screen of the appropriate size
    cv::Mat screen(screenSize, CV_8UC3, cv::Scalar(0, 0, 0)); // Black background
    // // Calculate the position to center the image on the screen
    // int x = (screenSize.width) / 2;
    // int y = (screenSize.height) / 2;

    // Copy the image onto the center of the screen
    cv::Rect roi(cv::Point(0, 0), image.size());
    image.copyTo(screen(roi));

    // Add text to the image
    // cv::putText(image, text1, position1, fontFace1, fontScale1, color1, thickness1);
    // Add the second text to the image
    cv::putText(image, text2, position2, fontFace2, fontScale2, color2, thickness2);
    // Add the second text to the image
    cv::putText(image, text3, position3, fontFace3, fontScale3, color3, thickness3);

    // Get the current path
    std::string  currentPath = "/home/ruan-x/workSpace_R/src/scan_recorder/picture";

    // Specify the filename for saving the image
    std::string filename = currentPath + "/Score result.png";
    cv::imwrite(filename, image);

    cv::namedWindow("Result Image", cv::WINDOW_NORMAL);
    cv::moveWindow("Result Image", 0, 0);
    // Display the image using OpenCV's imshow function
    cv::imshow("Result Image", image);
    cv::waitKey(0); // Wait for a key press

    // Release video capture and destroy windows
    cap.release();
    cv::destroyAllWindows();

    // std::cout << "spin start" << std::endl;
   while (ros::ok()) {
        //do notiong;
    }
    // ros::spin();
    return 0;
}
