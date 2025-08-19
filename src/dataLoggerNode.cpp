#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip> // For std::setprecision
#include <chrono>
#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <unistd.h>

// Custom message
#include "global_to_polar_cpp/msg/polar_grid.hpp"
#include "planning_custom_msgs/msg/path_with_velocity.hpp"

class DataLoggerNode : public rclcpp::Node
{
public:
    DataLoggerNode() : Node("data_logger_node"), scan_received_(false), grid_received_(false), path_received_(false)
    {
        // 매개변수 선언 및 가져오기
        bool enable_auto_filename = this->declare_parameter<bool>("enable_auto_filename", true);
        std::string map_name = this->declare_parameter<std::string>("map_name", "default_map");
        std::string auto_save_dir = this->declare_parameter<std::string>("auto_save_dir", "auto_save");
        
        // 파일 경로 생성
        if (enable_auto_filename) {
            output_csv_file_ = generateAutoFilename(map_name, auto_save_dir);
        } else {
            output_csv_file_ = this->declare_parameter<std::string>("output_csv_file", "");
            if (output_csv_file_.empty()) {
                RCLCPP_ERROR(this->get_logger(), "output_csv_file parameter is required when auto_filename is disabled. Shutting down.");
                rclcpp::shutdown();
                return;
            }
        }

        // 디렉토리 생성 (존재하지 않는 경우)
        std::filesystem::path file_path(output_csv_file_);
        std::filesystem::path dir_path = file_path.parent_path();
        if (!dir_path.empty() && !std::filesystem::exists(dir_path)) {
            std::filesystem::create_directories(dir_path);
            RCLCPP_INFO(this->get_logger(), "Created directory: %s", dir_path.c_str());
        }

        // CSV 파일 열기
        csv_file_.open(output_csv_file_, std::ios::out | std::ios::trunc);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s", output_csv_file_.c_str());
            rclcpp::shutdown();
            return;
        }

        // Write the header row to the CSV file
        writeHeader();

        // Subscribers
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&DataLoggerNode::laserScanCallback, this, std::placeholders::_1));

        polar_grid_sub_ = this->create_subscription<global_to_polar_cpp::msg::PolarGrid>(
            "/polar_grid", 10,
            std::bind(&DataLoggerNode::polarGridCallback, this, std::placeholders::_1));

        path_with_velocity_sub_ = this->create_subscription<planning_custom_msgs::msg::PathWithVelocity>(
            "/planned_path_with_velocity", 10,
            std::bind(&DataLoggerNode::pathWithVelocityCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Data Logger Node initialized. Logging to %s", output_csv_file_.c_str());
    }

    ~DataLoggerNode()
    {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

private:
    std::string generateAutoFilename(const std::string& map_name, const std::string& auto_save_dir)
    {
        try {
            // 소스 디렉토리 우선 사용 (개발용), 없으면 패키지 공유 디렉토리 사용
            std::string auto_save_path;
            
            // 현재 작업 디렉토리에서 소스 경로 확인
            char* cwd = getcwd(nullptr, 0);
            std::string current_dir(cwd);
            free(cwd);
            
            std::string src_auto_save_path = current_dir + "/src/global_to_polar_cpp/" + auto_save_dir;
            
            // 소스 디렉토리가 존재하면 사용, 없으면 install 디렉토리 사용
            if (std::filesystem::exists(src_auto_save_path)) {
                auto_save_path = src_auto_save_path;
                RCLCPP_INFO(this->get_logger(), "Using source directory for auto-save: %s", auto_save_path.c_str());
            } else {
                std::string package_share_dir = ament_index_cpp::get_package_share_directory("global_to_polar_cpp");
                auto_save_path = package_share_dir + "/" + auto_save_dir;
                RCLCPP_INFO(this->get_logger(), "Using install directory for auto-save: %s", auto_save_path.c_str());
            }
            
            // 현재 시간 가져오기
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) % 1000;
            
            // 시간 포맷팅 (YYYY-MM-DD_HH-MM-SS-mmm)
            std::stringstream ss;
            ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d_%H-%M-%S");
            ss << "-" << std::setfill('0') << std::setw(3) << ms.count();
            
            // 파일명 생성: map_name_timestamp.csv
            std::string filename = map_name + "_" + ss.str() + ".csv";
            
            // 전체 경로 생성
            std::string full_path = auto_save_path + "/" + filename;
            
            RCLCPP_INFO(this->get_logger(), "Auto-generated filename: %s", full_path.c_str());
            return full_path;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to generate auto filename: %s", e.what());
            // 폴백: 현재 디렉토리에 기본 파일명 사용
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d_%H-%M-%S");
            return map_name + "_" + ss.str() + ".csv";
        }
    }

    void writeHeader()
    {
        // LaserScan headers (assuming 1080 points)
        for (int i = 0; i < 1080; ++i) {
            csv_file_ << "scan_" << i << ",";
        }
        // PolarGrid headers (1080 points)
        for (int i = 0; i < 1080; ++i) {
            csv_file_ << "grid_" << i << ",";
        }
        // PathPointArray headers (16 points with x, y, v, yaw)
        for (int i = 0; i < 16; ++i) {
            csv_file_ << "x" << (i+1) << ",";
            csv_file_ << "y" << (i+1) << ",";
            csv_file_ << "v" << (i+1) << ",";
            csv_file_ << "yaw" << (i+1) << (i == 15 ? "" : ",");
        }
        csv_file_ << "\n";
    }

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // We assume the laser scan also has 1081 points.
        // If not, you might need to adjust the logic or header.
        if (msg->ranges.size() != 1080) {
             RCLCPP_WARN_ONCE(this->get_logger(), "Received LaserScan with %zu points, expected 1081. CSV columns might not align.", msg->ranges.size());
        }
        last_scan_ = msg;
        scan_received_ = true;
        writeData(); // Attempt to write data every time a new scan arrives
    }

    void polarGridCallback(const global_to_polar_cpp::msg::PolarGrid::SharedPtr msg)
    {
        if (msg->ranges.size() != 1080) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Received PolarGrid with %zu points, expected 1081. CSV columns might not align.", msg->ranges.size());
        }
        last_grid_ = msg;
        grid_received_ = true;
    }

    void pathWithVelocityCallback(const planning_custom_msgs::msg::PathWithVelocity::SharedPtr msg)
    {
        last_path_ = msg;
        path_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received PathWithVelocity with %zu points", msg->points.size());
    }

    void writeData()
    {
        // Only write to file if we have received at least one of each message type
        if (!scan_received_ || !grid_received_ || !path_received_) {
            return;
        }

        // Set precision for floating point numbers
        csv_file_ << std::fixed << std::setprecision(5);

        // Write LaserScan ranges
        for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
            csv_file_ << last_scan_->ranges[i] << ",";
        }

        // Write PolarGrid ranges
        for (size_t i = 0; i < last_grid_->ranges.size(); ++i) {
            csv_file_ << last_grid_->ranges[i] << ",";
        }

        // Write PathPointArray data (16 points with x, y, v, yaw)
        for (int i = 0; i < 16; ++i) {
            if (i < static_cast<int>(last_path_->points.size())) {
                const auto& point = last_path_->points[i];
                csv_file_ << point.x << ",";
                csv_file_ << point.y << ",";
                csv_file_ << point.velocity << ",";
                csv_file_ << point.yaw << (i == 15 ? "" : ",");
            } else {
                // Pad with zeros for missing points
                csv_file_ << "0,0,0,0" << (i == 15 ? "" : ",");
            }
        }
        csv_file_ << "\n";
        
        // Flush the buffer to ensure data is written to disk immediately
        csv_file_.flush();

        // Reset flags to ensure we wait for a new pair of messages
        scan_received_ = false;
        grid_received_ = false;
        path_received_ = false;
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<global_to_polar_cpp::msg::PolarGrid>::SharedPtr polar_grid_sub_;
    rclcpp::Subscription<planning_custom_msgs::msg::PathWithVelocity>::SharedPtr path_with_velocity_sub_;

    // Data storage
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    global_to_polar_cpp::msg::PolarGrid::SharedPtr last_grid_;
    planning_custom_msgs::msg::PathWithVelocity::SharedPtr last_path_;
    bool scan_received_;
    bool grid_received_;
    bool path_received_;

    // File handling
    std::ofstream csv_file_;
    std::string output_csv_file_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataLoggerNode>());
    rclcpp::shutdown();
    return 0;
}
