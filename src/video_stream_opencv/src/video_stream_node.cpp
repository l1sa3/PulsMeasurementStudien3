#include "rclcpp/rclcpp.hpp"
#include <nodelet/loader.h>

int main(int argc, char **argv)
{
    //ros::init(argc, argv, "video_stream");
    rclcpp:init(argc, argv)

    nodelet::Loader manager(true);
    nodelet::M_string remappings;
    nodelet::V_string my_argv(argv + 1, argv + argc);
    my_argv.push_back("--shutdown-on-close"); // Internal

    manager.load(ros::this_node::getName(), "video_stream_opencv/VideoStream", remappings, my_argv);

    ros::spin();

    return 0;
}
