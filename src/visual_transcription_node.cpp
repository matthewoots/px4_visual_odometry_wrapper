#include <visual_transcription_node.h>

void visual_transcription::visual_pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> visual_pose_lock(pose_mutex);
    
    // RCLCPP_INFO(this->get_logger(), " ", );
    visual_pose.translation() = Eigen::Vector3d(
        msg->pose.position.x, 
        msg->pose.position.y, 
        msg->pose.position.z
    );

    visual_pose.linear() = Eigen::Quaterniond(
        msg->pose.orientation.w,
        msg->pose.orientation.x, 
        msg->pose.orientation.y,
        msg->pose.orientation.z).toRotationMatrix();
    
    visual_pose.translation() = R * visual_pose.translation();
    visual_pose.linear() = R * visual_pose.linear() * R.inverse();
}

void visual_transcription::visual_odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> visual_pose_lock(pose_mutex);

    // RCLCPP_INFO(this->get_logger(), " ", );
    visual_pose.translation() = Eigen::Vector3d(
        msg->pose.pose.position.x, 
        msg->pose.pose.position.y, 
        msg->pose.pose.position.z
    );
    visual_pose.linear() = Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x, 
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z).toRotationMatrix();


    visual_pose.translation() = R * visual_pose.translation();
    visual_pose.linear() = R * visual_pose.linear() * R.inverse();
}

/** @brief Subscribe to timesync message **/
void visual_transcription::timesync_callback(
    const px4_msgs::msg::Timesync::SharedPtr msg)
{
    std::lock_guard<std::mutex> timestamp_lock(timestamp_mutex);

    _timestamp.store(msg->timestamp);
    
    timestamp_offset_start = std::chrono::steady_clock::now();
}

/** @brief Subscribe to timesync status message **/
void visual_transcription::timesync_status_callback(
    const px4_msgs::msg::TimesyncStatus::SharedPtr msg)
{
    // Round trip time needs to be halved (s)
    _message_latency.store((double)(msg->round_trip_time) / pow(10.0, 6) / 2);
    // Use estimated time offset (s)
    _timestamp_est_offset.store((double)(msg->estimated_offset) / pow(10.0, 6));
}

int32_t visual_transcription::update_duration_with_offset()
{
    std::lock_guard<std::mutex> timestamp_lock(timestamp_mutex);

    auto tstamp = std::chrono::steady_clock::now() - timestamp_offset_start;
    int32_t nsec = std::chrono::duration_cast<std::chrono::milliseconds>(tstamp).count();
    return nsec;
}

void visual_transcription::publish_timer()
{
    std::lock_guard<std::mutex> visual_pose_lock(pose_mutex);

    Eigen::Quaterniond q(visual_pose.linear());

    int32_t current_time_offset = update_duration_with_offset();

    // Do timesync here
    // rclcpp::Duration offset(
    //     std::chrono::duration<std::chrono::nanoseconds>(current_time_offset));

    // rclcpp::Time now = this->get_clock()->now();

    auto vo_msg = px4_msgs::msg::VehicleVisualOdometry();
    vo_msg.timestamp = _timestamp.load() + current_time_offset;
    vo_msg.timestamp_sample = _timestamp.load() + current_time_offset;

    vo_msg.local_frame = px4_msgs::msg::VehicleVisualOdometry::LOCAL_FRAME_NED;
    // uint8 LOCAL_FRAME_NED=0         # NED earth-fixed frame
    // uint8 LOCAL_FRAME_FRD=1         # FRD earth-fixed frame, arbitrary heading reference
    // uint8 LOCAL_FRAME_OTHER=2       # Not aligned with the std frames of reference
    // uint8 BODY_FRAME_FRD=3          # FRD body-fixed frame

    Eigen::Vector3d pose = visual_pose.translation();
    vo_msg.x = pose.x();
    vo_msg.y = pose.y();
    vo_msg.z = pose.z();

    // Quaternion rotation from FRD body frame to refernce frame
    vo_msg.q[0] = q.w(); // w
    vo_msg.q[1] = q.x(); // x
    vo_msg.q[2] = q.y(); // y
    vo_msg.q[3] = q.z(); // z

    vo_msg.q_offset[0] = NAN;
    
    vo_msg.pose_covariance[0] = NAN;
    vo_msg.pose_covariance[15] = NAN;
    
    vo_msg.velocity_frame = 0;

    vo_msg.vx = NAN;
    vo_msg.vy = NAN;
    vo_msg.vz = NAN;

    vo_msg.rollspeed = NAN;
    vo_msg.pitchspeed = NAN;
    vo_msg.yawspeed = NAN;
    
    vo_msg.velocity_covariance[0] = NAN;
    vo_msg.velocity_covariance[15] = NAN; 

    _visual_odometry_publisher->publish(vo_msg);
}

bool visual_transcription::setup_subscriber_and_timer(
    std::string pose_or_odom)
{
    if (pose_or_odom.compare("pose") != 0 && pose_or_odom.compare("odom") != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid message type! %s", pose_or_odom.c_str());
        return false;
    }
    
    // Check if subscription is successfull and then initiate timer
    RCLCPP_INFO(this->get_logger(), "Waiting for first visual odometry message");
    rclcpp::WaitSet wait_set;

    bool is_posestamped = (pose_or_odom.compare("pose") == 0);
    
    if (is_posestamped)
    {
        // visual message subscription
        _visual_sub_pose = 
            this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/visual/pose", 10, std::bind(&visual_transcription::visual_pose_callback, this, _1));
        wait_set.add_subscription(_visual_sub_pose);
    }
    else // odometry message
    {
        // visual message subscription
        _visual_sub_odom = 
            this->create_subscription<nav_msgs::msg::Odometry>(
            "/visual/pose", 10, std::bind(&visual_transcription::visual_odom_callback, this, _1));
        wait_set.add_subscription(_visual_sub_odom);
    }
    
    int wait_time_s = 10;
    auto ret = wait_set.wait(std::chrono::seconds(wait_time_s));
    if (ret.kind() == rclcpp::WaitResultKind::Ready) 
    {    
        rclcpp::MessageInfo info;
        if (is_posestamped)
        {
            geometry_msgs::msg::PoseStamped msg;
            auto ret_take = _visual_sub_pose->take(msg, info);
            if (ret_take) {
                RCLCPP_INFO(this->get_logger(), "Heard visual_message, initializing timer now");
                _timer = this->create_wall_timer(
                    std::chrono::milliseconds(_interval_milliseconds), std::bind(&visual_transcription::publish_timer, this));                
            } 
            else {
                RCLCPP_ERROR(this->get_logger(), "No message recieved from visual node");
                return false;
            }

            rclcpp::Time now = this->get_clock()->now();
            // Get first time offset from data
            rclcpp::Duration rclcpp_offset = now - msg.header.stamp;
        
            visual_to_pub_offset = 
                (rclcpp_offset.to_chrono<std::chrono::nanoseconds>()).count();
        }
        else
        {
            nav_msgs::msg::Odometry msg;
            auto ret_take = _visual_sub_odom->take(msg, info);
            if (ret_take) {
                RCLCPP_INFO(this->get_logger(), "Heard visual_message, initializing timer now");
                _timer = this->create_wall_timer(
                    std::chrono::milliseconds(_interval_milliseconds), std::bind(&visual_transcription::publish_timer, this));
            } 
            else {
                RCLCPP_ERROR(this->get_logger(), "No message recieved from visual node");
                return false;
            }

            rclcpp::Time now = this->get_clock()->now();
            // Get first time offset from data
            rclcpp::Duration rclcpp_offset = now - msg.header.stamp;
        
            visual_to_pub_offset = 
                (rclcpp_offset.to_chrono<std::chrono::nanoseconds>()).count();
        }
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Timeout after %ds, couldn't wait for visual message", wait_time_s);
        return false;
    }

    // nanoseconds to seconds = 10^-9
    // nanoseconds to milliseconds = 10^-6
    double milliseconds = (double)visual_to_pub_offset / pow(10,6);

    RCLCPP_INFO(this->get_logger(), "Offset initialized (now - visual_system) %.5lfms", milliseconds);


    return true;
}