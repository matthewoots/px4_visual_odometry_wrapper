#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/timesync_status.hpp>

#include <visual_transcription_node.h>

int main(int argc, char *argv[])
{
	std::cout << "Visual odometry wrapper node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	// Help from https://github.com/clalancette/mtexec_example
	// Under nav_node/src/nav_node.cpp
	rclcpp::executors::MultiThreadedExecutor executor;
  	auto vis = std::make_shared<visual_transcription>();
	executor.add_node(vis);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}