#include <chrono>
#include <cmath>
#include <set>
#include <string>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "aruco_opencv_msgs/msg/aruco_detection.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>

// image_transport (compressed subscription)
#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>

// TF2 (to transform marker pose from camera_link -> base_frame)
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// NEW: for robot pose in odom + yaw extraction
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using aruco_opencv_msgs::msg::ArucoDetection;

// ---------- helpers (minimal) ----------
static inline double normalizeAngle(double a)
{
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static inline double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(tq);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

class ArucoExplorerNode : public rclcpp::Node
{
public:
  ArucoExplorerNode()
  : Node("aruco_explorer_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    state_(State::SCANNING),
    current_target_index_(0)
  {
    // ---------------- Parameters ----------------
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

    // NEW: fixed frame to store marker positions during scan
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");

    scan_angular_vel_ = this->declare_parameter<double>("scan_angular_vel", 1.0);
    scan_duration_    = this->declare_parameter<double>("scan_duration", 40.0);

    // IMPORTANT: base image topic (NOT /compressed). image_transport will select compressed.
    image_topic_ = this->declare_parameter<std::string>("image_topic", "/camera/image");

    // Dictionary (ARUCO_ORIGINAL = 16)
    aruco_dictionary_id_ =
      this->declare_parameter<int>("aruco_dictionary_id", cv::aruco::DICT_ARUCO_ORIGINAL);

    // Control (X error only)
    pixel_angular_gain_  = this->declare_parameter<double>("pixel_angular_gain", 0.003);
    center_tolerance_px_ = this->declare_parameter<int>("center_tolerance_px", 10);
    search_angular_vel_  = this->declare_parameter<double>("search_angular_vel", 0.6);
    max_angular_vel_     = this->declare_parameter<double>("max_angular_vel", 0.8);

    aruco_dict_ = cv::aruco::getPredefinedDictionary(aruco_dictionary_id_);
    aruco_params_ = cv::aruco::DetectorParameters::create();
    aruco_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    // ---------------- Publishers ----------------
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Publish overlay ONLY when centered
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/aruco_explorer/marker_image", 10);

    // ---------------- Subscribers ----------------
    // 1) Tracker detections used during SCANNING to collect IDs + save yaw + save position
    detection_sub_ = this->create_subscription<ArucoDetection>(
      "aruco_detections", 10,
      std::bind(&ArucoExplorerNode::arucoCallback, this, std::placeholders::_1));

    // 2) Compressed image subscription via image_transport
    image_transport::TransportHints hints(this, "compressed");
    image_sub_ = image_transport::create_subscription(
      this,
      image_topic_,
      std::bind(&ArucoExplorerNode::imageCallback, this, std::placeholders::_1),
      hints.getTransport(),
      rclcpp::SensorDataQoS().get_rmw_qos_profile()
    );

    // ---------------- Timer ----------------
    scan_start_time_ = this->now();
    control_timer_ = this->create_wall_timer(
      50ms, std::bind(&ArucoExplorerNode::controlLoop, this));

    // begin scaning
    RCLCPP_INFO(this->get_logger(), "SCANNING for %.1f s ...", scan_duration_);
  }

private:
  enum class State { SCANNING, GO_TO_MARKER, DONE };

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string base_frame_;

  // NEW
  std::string odom_frame_;

  // Parameters
  double scan_angular_vel_{1.0};
  double scan_duration_{40.0};

  std::string image_topic_;
  int aruco_dictionary_id_{cv::aruco::DICT_ARUCO_ORIGINAL};

  double pixel_angular_gain_{0.003};
  int center_tolerance_px_{10};
  double search_angular_vel_{0.6};
  double max_angular_vel_{1.0};

  // OpenCV ArUco
  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;

  // ROS
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<ArucoDetection>::SharedPtr detection_sub_;
  image_transport::Subscriber image_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // State
  State state_;
  rclcpp::Time scan_start_time_;

  // Scanning results
  std::set<int> discovered_ids_;
  std::vector<int> sorted_ids_;
  std::size_t current_target_index_;

  // save yaw for each marker (in base_frame_) from scanning
  std::unordered_map<int, double> saved_yaw_base_;

  // NEW: store marker position (x,y) in ODOM from scanning
  std::unordered_map<int, std::pair<double,double>> saved_marker_xy_odom_;

  // Latest image
  sensor_msgs::msg::Image::ConstSharedPtr last_image_;

  // Tracking
  bool target_visible_{false};
  double target_err_x_px_{0.0};

  // Keep direction when marker disappears
  // (we’ll still keep the variable, but now direction comes from yaw-to-saved-position)
  int search_dir_{+1};

  // Marker draw info (circle around marker)
  double target_marker_cx_px_{0.0};
  double target_marker_cy_px_{0.0};
  double target_marker_radius_px_{60.0};

  // Log control (print "found..." once per target)
  int last_found_log_id_{-1};

  // ---------------- Callbacks ----------------

  void arucoCallback(const ArucoDetection::SharedPtr msg)
  {
    if (state_ != State::SCANNING) return;

    // msg->header.frame_id is camera_link
    for (const auto & m : msg->markers) {
      discovered_ids_.insert(m.marker_id);

      geometry_msgs::msg::PoseStamped cam_pose;
      cam_pose.header = msg->header;
      cam_pose.pose   = m.pose;

      try {
        // ---- keep your original saved yaw in base_link ----
        auto base_pose = tf_buffer_.transform(cam_pose, base_frame_, tf2::durationFromSec(0.05));
        const double x = base_pose.pose.position.x;
        const double y = base_pose.pose.position.y;
        const double yaw = std::atan2(y, x);
        saved_yaw_base_[m.marker_id] = yaw;

        // ---- NEW: save marker position in odom (fixed frame) ----
        auto odom_pose = tf_buffer_.transform(cam_pose, odom_frame_, tf2::durationFromSec(0.05));
        saved_marker_xy_odom_[m.marker_id] = {odom_pose.pose.position.x, odom_pose.pose.position.y};

      } catch (const tf2::TransformException &) {
        // keep logs clean
      }
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    last_image_ = msg;

    if (state_ != State::GO_TO_MARKER || current_target_index_ >= sorted_ids_.size()) {
      target_visible_ = false;
      return;
    }

    const int target_id = sorted_ids_[current_target_index_];

    try {
      auto cv_ptr = cv_bridge::toCvCopy(last_image_, "bgr8");
      const cv::Mat & img = cv_ptr->image;

      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;

      cv::aruco::detectMarkers(img, aruco_dict_, corners, ids, aruco_params_);

      target_visible_ = false;

      const float img_cx = img.cols * 0.5f;

      for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] != target_id) continue;

        const auto & c = corners[i];

        // Marker center = average corners
        float mx = 0.f, my = 0.f;
        for (const auto & p : c) { mx += p.x; my += p.y; }
        mx *= 0.25f;
        my *= 0.25f;

        target_marker_cx_px_ = mx;
        target_marker_cy_px_ = my;

        // X error (rotation only)
        target_err_x_px_ = static_cast<double>(mx - img_cx);
        target_visible_ = true;

        // radius estimate from corners
        double r = 0.0;
        for (const auto & p : c) {
          const double dx = p.x - mx;
          const double dy = p.y - my;
          r += std::sqrt(dx*dx + dy*dy);
        }
        r /= 4.0;
        target_marker_radius_px_ = std::max(20.0, r);

        if (last_found_log_id_ != target_id) {
          last_found_log_id_ = target_id;
          RCLCPP_INFO(this->get_logger(), "Marker %d found, going to center...", target_id);
        }
        return;
      }

    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "imageCallback error: %s", e.what());
      target_visible_ = false;
    }
  }

  // ---------------- Control ----------------

  void controlLoop()
  {
    switch (state_) {
      case State::SCANNING:     handleScanning(); break;
      case State::GO_TO_MARKER: handleGoToMarker(); break;
      case State::DONE:         stopRobot(); break;
    }
  }

  void handleScanning()
  {
    const double elapsed = (this->now() - scan_start_time_).seconds();

    if (elapsed < scan_duration_) {
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = scan_angular_vel_;
      cmd_pub_->publish(cmd);
      return;
    }

    stopRobot();

    if (discovered_ids_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Scanning finished: no markers found.");
      state_ = State::DONE;
      return;
    }

    sorted_ids_.assign(discovered_ids_.begin(), discovered_ids_.end());
    std::sort(sorted_ids_.begin(), sorted_ids_.end());

    RCLCPP_INFO(this->get_logger(), "Scanning finished. Found %zu markers:", sorted_ids_.size());
    for (int id : sorted_ids_) RCLCPP_INFO(this->get_logger(), "  - %d", id);

    current_target_index_ = 0;
    target_visible_ = false;
    last_found_log_id_ = -1;
    state_ = State::GO_TO_MARKER;

    const int first_id = sorted_ids_[0];

    // Keep your original "init direction from yaw" (optional)
    auto it = saved_yaw_base_.find(first_id);
    if (it != saved_yaw_base_.end()) {
      search_dir_ = (it->second >= 0.0) ? +1 : -1;
    }

    RCLCPP_INFO(this->get_logger(), "New target marker: ID %d", first_id);
  }

  void handleGoToMarker()
  {
    if (current_target_index_ >= sorted_ids_.size()) {
      RCLCPP_INFO(this->get_logger(), "All markers processed. DONE.");
      state_ = State::DONE;
      stopRobot();
      return;
    }

    const int target_id = sorted_ids_[current_target_index_];

    // --------- Marker NOT visible -> compute yaw from saved marker position in odom ----------
    if (!target_visible_) {
      geometry_msgs::msg::Twist cmd;

      auto it_pos = saved_marker_xy_odom_.find(target_id);
      if (it_pos == saved_marker_xy_odom_.end()) {
        // no saved position -> safest stop
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
        return;
      }

      try {
        // current robot pose in odom: odom <- base_link
        geometry_msgs::msg::TransformStamped t =
          tf_buffer_.lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);

        const double rx   = t.transform.translation.x;
        const double ry   = t.transform.translation.y;
        const double ryaw = yawFromQuat(t.transform.rotation);

        const double mx = it_pos->second.first;
        const double my = it_pos->second.second;

        // desired yaw in odom towards the saved marker position
        const double desired_yaw = std::atan2(my - ry, mx - rx);

        // shortest yaw error
        const double yaw_err = normalizeAngle(desired_yaw - ryaw);

        // choose shortest direction at constant search speed (same behavior)
        const double dir = (yaw_err >= 0.0) ? +1.0 : -1.0;
        cmd.angular.z = dir * search_angular_vel_;

      } catch (const tf2::TransformException &) {
        cmd.angular.z = 0.0;
      }

      cmd_pub_->publish(cmd);
      return;
    }

    // --------- Marker visible -> pixel centering ----------
    if (std::abs(target_err_x_px_) < center_tolerance_px_) {
      stopRobot();
      RCLCPP_INFO(this->get_logger(), "Target %d CENTERED (|err_x| < %d px).",
                  target_id, center_tolerance_px_);

      publishCircleOverlay(target_id);

      current_target_index_++;
      target_visible_ = false;
      last_found_log_id_ = -1;

      if (current_target_index_ < sorted_ids_.size()) {
        const int next_id = sorted_ids_[current_target_index_];
        RCLCPP_INFO(this->get_logger(), "New target marker: ID %d", next_id);
      } else {
        RCLCPP_INFO(this->get_logger(), "All markers processed. DONE.");
        state_ = State::DONE;
      }
      return;
    }

    geometry_msgs::msg::Twist cmd;

    // If it rotates the wrong way, flip the sign (+ instead of -)
    double w = -pixel_angular_gain_ * target_err_x_px_;
    w = std::clamp(w, -max_angular_vel_, max_angular_vel_);

    cmd.angular.z = w;
    cmd_pub_->publish(cmd);
  }

  void publishCircleOverlay(int target_id)
  {
    (void)target_id;

    if (!last_image_) return;

    try {
      auto cv_ptr = cv_bridge::toCvCopy(last_image_, "bgr8");
      auto & img = cv_ptr->image;

      cv::Point center((int)target_marker_cx_px_, (int)target_marker_cy_px_);
      int radius = (int)std::round(target_marker_radius_px_);
      cv::circle(img, center, radius, cv::Scalar(0, 255, 0), 3);

      auto out_msg = cv_ptr->toImageMsg();
      out_msg->header = last_image_->header;
      image_pub_->publish(*out_msg);

      RCLCPP_INFO(this->get_logger(), "Published /aruco_explorer/marker_image.");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "publishCircleOverlay failed: %s", e.what());
    }
  }

  void stopRobot()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoExplorerNode>());
  rclcpp::shutdown();
  return 0;
}

