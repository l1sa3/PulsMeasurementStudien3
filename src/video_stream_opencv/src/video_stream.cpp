/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2016, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Sammy Pfeiffer
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <stdexcept>
#include <boost/filesystem.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/thread/thread.hpp>

// #include <video_stream_opencv/VideoStreamConfig.h>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace video_stream_opencv 
{

struct VideoStreamConfig
{
  std::string camera_name = "camera";
  std::string video_stream_provider = "0";
  std::string frame_id = "camera";
  std::string camera_info_url = "";
  std::string output_encoding = "bgr8";

  double set_camera_fps = 30.0;
  double fps = 30.0;
  double brightness = 0.5019607843137255;
  double contrast = 0.12549019607843137;
  double hue = 0.5;
  double saturation = 0.64;
  double exposure = 0.5;

  int buffer_queue_size = 100;
  int width = 0;
  int height = 0;

  bool flip_horizontal = false;
  bool flip_vertical = false;
  bool auto_exposure = true;
  bool loop_videofile = false;
  bool reopen_on_read_failure = false;
  bool always_subscribe = false;
};  

class VideoStream: public rclcpp::Node {
  public:
    explicit VideoStream(const rclcpp::NodeOptions & options)
    : rclcpp::Node("video_stream", options),
      subscriber_num_(0),
      capture_thread_running_(false)
    {
      declare_parameters();
      load_parameters();
      detect_provider_type();

      param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&VideoStream::config_callback, this, std::placeholders::_1));

      connection_check_timer_ = this->create_wall_timer(
        500ms, std::bind(&VideoStream::connection_timer_callback, this));

      RCLCPP_INFO(this->get_logger(), "video_stream component initialized");
    }

    ~VideoStream() override
    {
      stop_capture();
    }

  private:
    void declare_parameters()
    {
      this->declare_parameter<std::string>("camera_name", "camera");
      this->declare_parameter<std::string>("video_stream_provider", "0");
      this->declare_parameter<double>("set_camera_fps", 30.0);
      this->declare_parameter<int>("buffer_queue_size", 100);
      this->declare_parameter<double>("fps", 240.0);
      this->declare_parameter<std::string>("frame_id", "camera");
      this->declare_parameter<std::string>("camera_info_url", "");
      this->declare_parameter<bool>("flip_horizontal", false);
      this->declare_parameter<bool>("flip_vertical", false);
      this->declare_parameter<int>("width", 0);
      this->declare_parameter<int>("height", 0);
      this->declare_parameter<double>("brightness", 0.5019607843137255);
      this->declare_parameter<double>("contrast", 0.12549019607843137);
      this->declare_parameter<double>("hue", 0.5);
      this->declare_parameter<double>("saturation", 0.64);
      this->declare_parameter<bool>("auto_exposure", true);
      this->declare_parameter<double>("exposure", 0.5);
      this->declare_parameter<bool>("loop_videofile", false);
      this->declare_parameter<bool>("reopen_on_read_failure", false);
      this->declare_parameter<std::string>("output_encoding", "bgr8");
      this->declare_parameter<bool>("always_subscribe", false);
    }
    
    void load_parameters()
    {
      camera_name_ = this->get_parameter("camera_name").as_string();
      video_stream_provider_ = this->get_parameter("video_stream_provider").as_string();
      set_camera_fps_ = this->get_parameter("set_camera_fps").as_double();
      buffer_queue_size_ = this->get_parameter("buffer_queue_size").as_int();
      fps_ = this->get_parameter("fps").as_double();
      frame_id_ = this->get_parameter("frame_id").as_string();
      camera_info_url_ = this->get_parameter("camera_info_url").as_string();
      flip_horizontal_ = this->get_parameter("flip_horizontal").as_bool();
      flip_vertical_ = this->get_parameter("flip_vertical").as_bool();
      width_ = this->get_parameter("width").as_int();
      height_ = this->get_parameter("height").as_int();
      brightness_ = this->get_parameter("brightness").as_double();
      contrast_ = this->get_parameter("contrast").as_double();
      hue_ = this->get_parameter("hue").as_double();
      saturation_ = this->get_parameter("saturation").as_double();
      auto_exposure_ = this->get_parameter("auto_exposure").as_bool();
      exposure_ = this->get_parameter("exposure").as_double();
      loop_videofile_ = this->get_parameter("loop_videofile").as_bool();
      reopen_on_read_failure_ = this->get_parameter("reopen_on_read_failure").as_bool();
      output_encoding_ = this->get_parameter("output_encoding").as_string();
      always_subscribe_ = this->get_parameter("always_subscribe").as_bool();

      if (fps_ > set_camera_fps_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Asked to publish at 'fps' %3f which is higher than the 'set_camera_fps' (%f), we can't publish faster than the camera provides images.",
        fps_, set_camera_fps_);
      fps_ = set_camera_fps_;
      }
    }

    void detect_provider_type()
    {
      try{
        (void)std::stoi(video_stream_provider_);
        video_stream_provider_type_ = "videodevice";
        return;
      } catch (const std::invalid_argument &) {
        // not a plain integer, continue with URL/file detection
      } catch (const std::out_of_range &) {
        // not a usable device number, continue with URL/file detection
      }

      if (video_stream_provider_.find("http://") != std::string::npos ||
          video_stream_provider_.find("https://") != std::string::npos) {
        video_stream_provider_type_ = "http_stream";
      } else if (video_stream_provider_.find("rtsp://") != std::string::npos) {
        video_stream_provider_type_ = "rtsp_stream";
      } else {
        std::error_code ec;
        auto status = fs::status(video_stream_provider_, ec);
        if (!ec) {
          if (fs::is_character_file(status) || fs::is_block_file(status)) {
            video_stream_provider_type_ = "videodevice";
          } else if (fs::is_regular_file(status)) {
            video_stream_provider_type_ = "videofile";
          } else {
            video_stream_provider_type_ = "unknown";
          }
        } else {
          video_stream_provider_type_ = "unknown";
        }
      }
    }

    void initialize_publishers()
    {
      cam_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
        this, camera_name_, camera_info_url_);

      cam_info_msg_ = cam_info_manager_->getCameraInfo();
      cam_info_msg_.header.frame_id = frame_id_;
      
      image_transport::ImageTransport it(shared_from_this());
      pub_ = it.advertiseCamera("image_raw", 1);

    }

    sensor_msgs::msg::CameraInfo get_default_camera_info_from_image(
      const sensor_msgs::msg::Image::SharedPtr & img)
    {
      sensor_msgs::msg::CameraInfo cam_info_msg;
      cam_info_msg.header.frame_id = img->header.frame_id;
      // Fill image size
      cam_info_msg.height = img->height;
      cam_info_msg.width = img->width;
      RCLCPP_INFO(this->get_logger(), "The image width is: %u", img->width);
      RCLCPP_INFO(this->get_logger(), "The image height is: %u", img->height);
      // Add the most common distortion model as sensor_msgs/CameraInfo says
      cam_info_msg.distortion_model = "plumb_bob";
      // Don't let distorsion matrix be empty
      cam_info_msg.d = std::vector<double>(5, 0.0);
      // Give a reasonable default intrinsic camera matrix
      cam_info_msg.k = {
        1.0, 0.0, static_cast<double>(img->width) / 2.0,
        0.0, 1.0, static_cast<double>(img->height) / 2.0,
        0.0, 0.0, 1.0
      };
      // Give a reasonable default rectification matrix
      cam_info_msg.r = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
      };
      // Give a reasonable default projection matrix
      cam_info_msg.p = {
        1.0, 0.0, static_cast<double>(img->width) / 2.0, 0.0,
        0.0, 1.0, static_cast<double>(img->height) / 2.0, 0.0,
        0.0, 0.0, 1.0, 0.0
      };
      return cam_info_msg;
    }

    void connection_timer_callback()
    {
      std::lock_guard<std::mutex> lock(s_mutex_);

      size_t image_subs = pub_.getNumSubscribers();
      size_t info_subs = pub_.getNumSubscribers();
      int current_total = static_cast<int>(image_subs + info_subs);

      if ((current_total > 0 || always_subscribe_) && subscriber_num_ == 0) {
        subscriber_num_ = current_total > 0 ? current_total : 1;
        subscribe();
        return;
      }

      if (!always_subscribe_ && video_stream_provider_type_ != "videofile" && current_total == 0 && subscriber_num_ > 0) {
        subscriber_num_ = 0;
        unsubscribe();
        return;
      }

      subscriber_num_ = current_total;
    }

  void subscribe()
  {
    RCLCPP_DEBUG(this->get_logger(), "Subscribe");
    std::lock_guard<std::mutex> lock(c_mutex_);

    if (cap_ && cap_->isOpened()) {
      return;
    }
    // initialize camera info publisher
    initialize_publishers();
    // cam_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    //   this, camera_name_, camera_info_url_);
    // // Get the saved camera info if any
    // cam_info_msg_ = cam_info_manager_->getCameraInfo();
    // cam_info_msg_.header.frame_id = frame_id_;

    cap_ = std::make_shared<cv::VideoCapture>();
    // initialize camera
    try {
      int device_num = std::stoi(video_stream_provider_);
      RCLCPP_INFO(this->get_logger(), "Opening VideoCapture with provider: /dev/video%d", device_num);
      cap_->open(device_num);
    } catch (const std::invalid_argument &) {
      RCLCPP_INFO(this->get_logger(), "Opening VideoCapture with provider: %s", video_stream_provider_.c_str());
      cap_->open(video_stream_provider_);
    } catch (const std::out_of_range &) {
      RCLCPP_INFO(this->get_logger(), "Opening VideoCapture with provider: %s", video_stream_provider_.c_str());
      cap_->open(video_stream_provider_);
    }

    if (!cap_->isOpened()) {
      RCLCPP_FATAL(this->get_logger(), "Invalid 'video_stream_provider': %s", video_stream_provider_.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Video stream provider type detected: %s", video_stream_provider_type_.c_str());

    // OpenCV 2.4 returns -1 (instead of a 0 as the spec says) and prompts an error
    // HIGHGUI ERROR: V4L2: Unable to get property <unknown property string>(5) - Invalid argument
    double reported_camera_fps = cap_->get(cv::CAP_PROP_FPS);
    
    if (reported_camera_fps > 0.0) {
      RCLCPP_INFO(this->get_logger(), "Camera reports FPS: %.3f", reported_camera_fps);
    } else {
      RCLCPP_INFO(this->get_logger(), "Backend can't provide camera FPS information");
    }

    cap_->set(cv::CAP_PROP_FPS, set_camera_fps_);

    if (width_ != 0 && height_ != 0) {
      cap_->set(cv::CAP_PROP_FRAME_WIDTH, width_);
      cap_->set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    }

    cap_->set(cv::CAP_PROP_BRIGHTNESS, brightness_);
    cap_->set(cv::CAP_PROP_CONTRAST, contrast_);
    cap_->set(cv::CAP_PROP_HUE, hue_);
    cap_->set(cv::CAP_PROP_SATURATION, saturation_);

    if (auto_exposure_) {
      cap_->set(cv::CAP_PROP_AUTO_EXPOSURE, 0.75);
      exposure_ = 0.5;
    } else {
      cap_->set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
      cap_->set(cv::CAP_PROP_EXPOSURE, exposure_);
    }

    try {
      capture_thread_running_ = true;
      capture_thread_ = std::thread(&VideoStream::do_capture, this);

      auto period = std::chrono::duration<double>(1.0 / std::max(1.0, fps_));
      publish_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&VideoStream::do_publish, this));
    } catch (const std::exception & e) {
      capture_thread_running_ = false;
      RCLCPP_ERROR(this->get_logger(), "Failed to start capture thread or timer: %s", e.what());
      if (capture_thread_.joinable()) {
        capture_thread_.join();
      }
      if (cap_) {
        cap_->release();
        cap_.reset();
      }
      return;
    }
  }

  void unsubscribe()
  {
    RCLCPP_DEBUG(this->get_logger(), "Unsubscribe");
    stop_capture();
  }

  void stop_capture()
  {
    if (publish_timer_) {
      publish_timer_->cancel();
      publish_timer_.reset();
    }

    capture_thread_running_ = false;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }

    std::lock_guard<std::mutex> lock(q_mutex_);
    while (!frames_queue_.empty()) {
      frames_queue_.pop();
    }

    if (cap_) {
      cap_->release();
      cap_.reset();
    }
  }

  void do_capture()
  {
    RCLCPP_DEBUG(this->get_logger(), "Capture thread started");
    cv::Mat frame;
    int frame_counter = 0;

    while (rclcpp::ok() && capture_thread_running_ && subscriber_num_ > 0) {
      VideoStreamConfig latest_config;
      {
        std::lock_guard<std::mutex> lock(c_mutex_);
        latest_config.camera_name = camera_name_;
        latest_config.video_stream_provider = video_stream_provider_;
        latest_config.frame_id = frame_id_;
        latest_config.camera_info_url = camera_info_url_;
        latest_config.output_encoding = output_encoding_;
        latest_config.set_camera_fps = set_camera_fps_;
        latest_config.fps = fps_;
        latest_config.brightness = brightness_;
        latest_config.contrast = contrast_;
        latest_config.hue = hue_;
        latest_config.saturation = saturation_;
        latest_config.exposure = exposure_;
        latest_config.buffer_queue_size = buffer_queue_size_;
        latest_config.width = width_;
        latest_config.height = height_;
        latest_config.flip_horizontal = flip_horizontal_;
        latest_config.flip_vertical = flip_vertical_;
        latest_config.auto_exposure = auto_exposure_;
        latest_config.loop_videofile = loop_videofile_;
        latest_config.reopen_on_read_failure = reopen_on_read_failure_;
        latest_config.always_subscribe = always_subscribe_;
        // latest_config = config_;
      }

      if (!cap_ || !cap_->isOpened()) {
        RCLCPP_WARN(this->get_logger(), "Waiting for device...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      if (!cap_->read(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Could not capture frame");
        if (latest_config.reopen_on_read_failure) {
          RCLCPP_WARN(this->get_logger(), "Trying to reopen the device");
          unsubscribe();
          subscribe();
          continue;
        }
      }

      frame_counter++;

      if (video_stream_provider_type_ == "videofile") {
        std::this_thread::sleep_for(
          std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / std::max(1.0, latest_config.set_camera_fps))));
      }

      if (video_stream_provider_type_ == "videofile" &&
          frame_counter == static_cast<int>(cap_->get(cv::CAP_PROP_FRAME_COUNT))) {
        if (latest_config.loop_videofile) {
          cap_->open(video_stream_provider_);
          frame_counter = 0;
        } else {
          RCLCPP_INFO(this->get_logger(), "Reached the end of frames");
          break;
        }
      }

      if (!frame.empty()) {
        std::lock_guard<std::mutex> g(q_mutex_);
        // accumulate only until max_queue_size
        while (static_cast<int>(frames_queue_.size()) > latest_config.buffer_queue_size) {
          frames_queue_.pop();
        }
        frames_queue_.push(frame.clone());
      }
    }

    RCLCPP_DEBUG(this->get_logger(), "Capture thread finished");
  }

  void do_publish()
  {
    cv::Mat frame;
    bool is_new_image = false;

    {
      std::lock_guard<std::mutex> g(q_mutex_);
      if (!frames_queue_.empty()) {
        frame = frames_queue_.front();
        frames_queue_.pop();
        is_new_image = true;
      }
    }

    // Check if grabbed frame is actually filled with some content
    if (frame.empty()) {
      return;
    }
    // From http://docs.opencv.org/modules/core/doc/operations_on_arrays.html#void flip(InputArray src, OutputArray dst, int flipCode)
    // FLIP_HORIZONTAL == 1, FLIP_VERTICAL == 0 or FLIP_BOTH == -1
    // Flip the image if necessary
    if (is_new_image) {
      if (flip_horizontal_ && flip_vertical_) {
        cv::flip(frame, frame, -1);
      } else if (flip_horizontal_) {
        cv::flip(frame, frame, 1);
      } else if (flip_vertical_) {
        cv::flip(frame, frame, 0);
      }
    }

    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = frame_id_;

    auto cv_image = std::make_shared<cv_bridge::CvImage>(header, "bgr8", frame);

    if (output_encoding_ != "bgr8") {
      try {
        // https://github.com/ros-perception/vision_opencv/blob/melodic/cv_bridge/include/cv_bridge/cv_bridge.h#L247
        cv_image = cv_bridge::cvtColor(cv_image, output_encoding_);
      } catch (const std::runtime_error & ex) {
        RCLCPP_ERROR(this->get_logger(), "Cannot change encoding to %s: %s",
                    output_encoding_.c_str(), ex.what());
        return;
      }
    }

    auto msg = cv_image->toImageMsg();

    if (cam_info_msg_.distortion_model.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "No calibration file given, publishing a reasonable default camera info.");
      cam_info_msg_ = get_default_camera_info_from_image(msg);
    }

    cam_info_msg_.header.stamp = header.stamp;
    cam_info_msg_.header.frame_id = frame_id_;
    // The timestamps are in sync thanks to this publisher
    pub_.publish(*msg, cam_info_msg_);
  }

  rcl_interfaces::msg::SetParametersResult config_callback(
  const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    bool needs_restart = false;
    bool show_forced_size = false;

    for (const auto & param : parameters) {
      const auto & name = param.get_name();

      if (name == "fps" && param.as_double() < 0.0) {
        result.successful = false;
        result.reason = "fps must be >= 0.0";
        return result;
      }

      if (name == "set_camera_fps" && param.as_double() < 0.0) {
        result.successful = false;
        result.reason = "set_camera_fps must be >= 0.0";
        return result;
      }

      if (name == "buffer_queue_size" && param.as_int() < 1) {
        result.successful = false;
        result.reason = "buffer_queue_size must be >= 1";
        return result;
      }

      if (name == "camera_name" ||
          name == "camera_info_url" ||
          name == "set_camera_fps" ||
          name == "width" ||
          name == "height" ||
          name == "video_stream_provider") {
        needs_restart = true;
      }

      if (name == "width" || name == "height") {
        show_forced_size = true;
      }
    }

    load_parameters();
    detect_provider_type();

    RCLCPP_INFO(this->get_logger(), "Camera name: %s", camera_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Provided camera_info_url: '%s'", camera_info_url_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing with frame_id: %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Setting camera FPS to: %.3f", set_camera_fps_);
    RCLCPP_INFO(this->get_logger(), "Throttling to fps: %.3f", fps_);
    RCLCPP_INFO(this->get_logger(), "Setting buffer size for capturing frames to: %d", buffer_queue_size_);
    RCLCPP_INFO(this->get_logger(), "Flip horizontal image is: %s", flip_horizontal_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Flip vertical image is: %s", flip_vertical_ ? "true" : "false");

    if (width_ != 0 && height_ != 0 && show_forced_size) {
      RCLCPP_INFO(this->get_logger(), "Forced image width is: %d", width_);
      RCLCPP_INFO(this->get_logger(), "Forced image height is: %d", height_);
    }

    if (subscriber_num_ > 0 && needs_restart) {
      unsubscribe();
      subscribe();
    }

    return result;
  }
  

  image_transport::CameraPublisher pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr connection_check_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;
  sensor_msgs::msg::CameraInfo cam_info_msg_;

  std::mutex q_mutex_;
  std::mutex s_mutex_;
  std::mutex c_mutex_;

  std::queue<cv::Mat> frames_queue_;
  std::shared_ptr<cv::VideoCapture> cap_;
  std::thread capture_thread_;
  std::atomic<bool> capture_thread_running_{false};

  int subscriber_num_;

  std::string camera_name_;
  std::string video_stream_provider_;
  std::string video_stream_provider_type_;
  std::string frame_id_;
  std::string camera_info_url_;
  std::string output_encoding_;

  double set_camera_fps_;
  double fps_;
  double brightness_;
  double contrast_;
  double hue_;
  double saturation_;
  double exposure_;

  int buffer_queue_size_;
  int width_;
  int height_;

  bool flip_horizontal_;
  bool flip_vertical_;
  bool auto_exposure_;
  bool loop_videofile_;
  bool reopen_on_read_failure_;
  bool always_subscribe_;
};

}  // namespace video_stream_opencv

RCLCPP_COMPONENTS_REGISTER_NODE(video_stream_opencv::VideoStream)