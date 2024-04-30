# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from std_msgs.msg import String

class my_Node(Node):
    def __init__(self):
        super().__init__("my_node")
        self.subscription = self.create_subscription(
            String, "my_topic", self.listener_callback, 10
        )
    def listener_callback(self, msg):
        self.get_logger().info(msg.string)


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            Range, "topic", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        if(msg.range > 0.5):
            self.get_logger().info(f"Black Line {msg.range}")
        else:
            self.get_logger().info(f"No Line {msg.range}")
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    #my_node = my_Node()

    rclpy.spin(minimal_subscriber)

    #rclpy.spin(my_node)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #my_node.destroy_node()

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
