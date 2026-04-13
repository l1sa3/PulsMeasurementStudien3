from pulse_interfaces.msg import Pulse
from datetime import datetime
from builtin_interfaces.msg import Time

import csv
import os
import rclpy


class PulsePublisher:

    def __init__(self, node, name):
        self.node = node
        self.name = name
        self.topic = "/" + name
        self.publisher = self.node.create_publisher(Pulse, self.topic, 10)
        self.sequence = 0
        self.date = datetime.now().strftime('%Y-%m-%d_%H:%M:%S')

    def publish(self, pulse, timestamp):
        """
        Publishes the pulse value to ROS and also writes it into a csv file.
        :param pulse: The pulse value to publish in ROS and to write into a csv file.
        :param timestamp: The timestamp corresponding to the pulse value.
        """
        self.node.get_logger().info("[PulsePublisher] Publishing pulse ('" + self.topic + "'): " + str(pulse))
        self.publish_to_ros(pulse, timestamp)
        self.write_to_csv(pulse, timestamp)

    def publish_to_ros(self, pulse, timestamp):
        """
        Publishes the pulse value to ROS using self.topic as topic.
        :param pulse: The pulse value to publish in ROS.
        :param timestamp: The timestamp corresponding to the pulse value.
        """
        ros_msg = Pulse()
        ros_msg.pulse = float(pulse)
        
        # ROS1-compatible: either float or rclpy.Time
        if isinstance(timestamp, (int, float)):
            sec = int (timestamp)
            nanosec = int ((timestamp - sec) * 1e9)
        else:
            # rclpy.Time -> seconds/nanoseconds
            time_msg = timestamp.to_msg()
            sec = time_msg.sec
            nanosec = time_msg.nanosec

        ros_msg.time.stamp.sec = sec
        ros_msg.time.stamp.nanosec = nanosec
        ros_msg.time.frame_id = ""  # optional

        self.publisher.publish(ros_msg)
        self.sequence += 1

    def write_to_csv(self, pulse, timestamp):
        """
        Writes the pulse value into a csv file.
        The file will be created inside the ROS_HOME directory using the name of the publisher
        and the start date of the script.
        :param pulse: The pulse value to write.
        :param timestamp: The timestamp corresponding to the pulse value.
        """
        filename = "pulse_measurement/" + self.name + "/pulses_" + self.date + ".csv"

        if not os.path.exists(os.path.dirname(filename)):
            os.makedirs(os.path.dirname(filename))

        csv_file = open(filename, "a+")
        writer = csv.writer(csv_file)
        writer.writerow([timestamp, pulse])
