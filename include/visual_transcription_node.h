#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>

#include <chrono>
#include <iostream>

using std::placeholders::_1;

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class visual_transcription : public rclcpp::Node 
{
    public:

	    visual_transcription() : Node("visual_transcription") 
        {
            _visual_odometry_publisher = 
                this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>
                ("fmu/vehicle_visual_odometry/in", 10);
            
            // fmu/timesync/in gives the timestamp from ros2 to px4
            // fmu/timesync/out outputs the corrected timestamp

            // get common timestamp
            _timesync_sub =
                this->create_subscription<px4_msgs::msg::Timesync>(
                "fmu/timesync/out", 10, std::bind(&visual_transcription::timesync_callback, this, _1));
            // get timestamp status
            _timesync_status_sub =
                this->create_subscription<px4_msgs::msg::TimesyncStatus>(
                "/timesync_status", 10, std::bind(&visual_transcription::timesync_status_callback, this, _1));
        }
    
    private:
	    rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr _visual_odometry_publisher;
        rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr _timesync_sub;
        rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr _timesync_status_sub;

        // Atomic to handle multithreading access to variable
        std::atomic<uint64_t> _timestamp;   // common synced timestamped

        /** @brief Subscribe to timesync message **/
        void timesync_callback(const px4_msgs::msg::Timesync::SharedPtr msg)
        {
            _timestamp.store(msg->timestamp);
        }

        /** @brief Subscribe to timesync status message **/
        void timesync_status_callback(const px4_msgs::msg::TimesyncStatus::SharedPtr msg) const
        {

        }

};