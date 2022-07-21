#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Geometry>

#include <chrono>
#include <iostream>
#include <mutex>

using std::placeholders::_1;

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class visual_transcription : public rclcpp::Node 
{
    public:

	    visual_transcription() : Node("visual_transcription") 
        {
            // Get parameters
            // frame = nwu, enu, ned, edn (image frame)
            // clock = steady-clock or system-clock
            this->declare_parameter<std::string>("frame", "nwu");
            this->declare_parameter<std::string>("clock", "steady-clock");
            this->declare_parameter<std::string>("msg_type", "pose");
            this->declare_parameter<std::string>("uav_prefix", "fmu");
            this->declare_parameter<int>("rate", 20);
            this->get_parameter("frame", _frame);
            this->get_parameter("clock", _clock);
            this->get_parameter("msg_type", _msg_type);
            this->get_parameter("uav_prefix", _uav_prefix);
            this->get_parameter("rate", _rate);

            _interval_milliseconds = (int)round((1 / _rate) * 1000); 

            _visual_odometry_publisher = 
                this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>
                ("fmu/vehicle_visual_odometry/in", 10);
            
            // fmu/timesync/in gives the timestamp from ros2 to px4
            // fmu/timesync/out outputs the corrected timestamp

            // get common timestamp subscription
            _timesync_sub =
                this->create_subscription<px4_msgs::msg::Timesync>(
                "/" + _uav_prefix + "/timesync/out", 10, std::bind(&visual_transcription::timesync_callback, this, _1));
            
            // get timestamp status subscription
            _timesync_status_sub =
                this->create_subscription<px4_msgs::msg::TimesyncStatus>(
                "/timesync_status", 10, std::bind(&visual_transcription::timesync_status_callback, this, _1));

            if (!setup_subscriber_and_timer(_msg_type))
                return;

            // https://github.com/chengguizi/basalt-mirror/blob/master/src/ros_live_vio.cpp#L297
            if (_frame.compare("nwu") == 0) // nwu to ned
                R << 1, 0, 0,
                    0, -1, 0,
                    0, 0, -1;
            else if (_frame.compare("enu") == 0) // enu to ned
                R << 0, 1, 0,
                    1, 0, 0,
                    0, 0, -1;
            else if (_frame.compare("ned") == 0) // enu to ned
                R << 1, 0, 0,
                    0, 1, 0,
                    0, 0, 1;
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Sorry, frame %s not supported", _frame.c_str());
                return;
            } 

        }
    
    private:

	    rclcpp::TimerBase::SharedPtr _timer;

        rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr _visual_odometry_publisher;
        
        rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr _timesync_sub;
        rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr _timesync_status_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _visual_sub_pose;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _visual_sub_odom;

        // Atomic to handle multithreading access to variable
        std::atomic<uint64_t> _timestamp;   // milliseconds common synced timestamped
        std::atomic<double> _timestamp_est_offset;
        std::atomic<double> _message_latency;

        int _rate, _interval_milliseconds;

        std::string _frame, _clock, _msg_type, _uav_prefix;

        Eigen::Affine3d visual_pose;

        int32_t visual_to_pub_offset; // nanoseconds

        std::chrono::time_point<std::chrono::steady_clock> timestamp_offset_start;

        std::mutex timestamp_mutex, pose_mutex;

        Eigen::Matrix3d R;

        void visual_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        
        void visual_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        void timesync_callback(const px4_msgs::msg::Timesync::SharedPtr msg);

        void timesync_status_callback(const px4_msgs::msg::TimesyncStatus::SharedPtr msg);

        int32_t update_duration_with_offset();

        void publish_timer();

        bool setup_subscriber_and_timer(std::string pose_or_odom);

};

// px4_msgs::msg::TimesyncStatus
// uint64 timestamp			# time since system start (microseconds)

// uint8 SOURCE_PROTOCOL_MAVLINK = 0
// uint8 SOURCE_PROTOCOL_RTPS = 1
// uint8 source_protocol			# timesync source. Source can be MAVLink or the microRTPS bridge

// uint64 remote_timestamp			# remote system timestamp (microseconds)
// int64 observed_offset			# raw time offset directly observed from this timesync packet (microseconds)
// int64 estimated_offset			# smoothed time offset between companion system and PX4 (microseconds)
// uint32 round_trip_time			# round trip time of this timesync packet (microseconds)