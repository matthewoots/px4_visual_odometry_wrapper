#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/timesync_status.hpp>

#include <visual_transcription_node.h>

int main(int argc, char *argv[])
{
	std::cout << "Visual odometry wrapper node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<visual_transcription>());

	rclcpp::shutdown();
	return 0;
}