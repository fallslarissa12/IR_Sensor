import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

from pyfirmata import Arduino, util

# Robot's speed when following the line
LINEAR_SPEED = 0.2

# Proportional constant to be applied on speed when turning 
# (Multiplied by the error value)
KP = 1.5/100 

# Global vars. initial values
# image_input = 0
# error = 0
# just_seen_line = False
# just_seen_right_mark = False
# should_move = False
# right_mark_count = 0
# finalization_countdown = None


# def start_follower_callback(request, response):
#     """
#     Start the robot.
#     In other words, allow it to move (again)
#     """
#     global should_move
#     global right_mark_count
#     global finalization_countdown
#     should_move = True
#     right_mark_count = 0
#     finalization_countdown = None
#     return response

# def stop_follower_callback(request, response):
#     """
#     Stop the robot
#     """
#     global should_move
#     global finalization_countdown
#     should_move = False
#     finalization_countdown = None
#     return response

board = Arduino("/dev/ttyACM1")

it = util.Iterator(board)
it.start()

ir_sensor0 = board.analog[0]
ir_sensor1 = board.analog[1]
ir_sensor2 = board.analog[2]
ir_sensor3 = board.analog[3]
ir_sensor4 = board.analog[4]
ir_sensor5 = board.analog[5]




ir_sensor0.enable_reporting()
ir_sensor1.enable_reporting()
ir_sensor2.enable_reporting()
ir_sensor3.enable_reporting()
ir_sensor4.enable_reporting()
ir_sensor5.enable_reporting()


def get_range(ir_sensor):
    val = ir_sensor.read()
    # find centimeter conversion formula in johnny five: https://github.com/rwaldron/johnny-five/blob/main/lib/proximity.js#L62
    # arduino analog reads voltage as integers between 0 to 1023
    # pyfirmata linearly translates the range to 0 to 1, so *1023
    #range_cm = 2076 / (val * 1023 - 11)
    return val


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(Range, "topic", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Range()
        # use message parameters from: http://wiki.ros.org/rosserial_arduino/Tutorials/IR%20Ranger
        # For GP2Y0A41SK0F only. Adjust for other IR rangers
        msg.radiation_type = 1
        msg.field_of_view = 0.01
        msg.min_range = 0.04
        msg.max_range = 0.3
        range_cm0 = get_range(ir_sensor0)
        msg.range = range_cm0  # in meters
        range_cm1 = get_range(ir_sensor1)
        range_cm2 = get_range(ir_sensor2)
        range_cm3 = get_range(ir_sensor3)
        range_cm4 = get_range(ir_sensor4)
        range_cm5 = get_range(ir_sensor5)



        message = Twist()

        #move forward straight (only)
        # message.linear.x = 2.0
#  
        self.publisher_.publish(msg)
        if((range_cm0 > 0.5) & (range_cm1 > 0.5) & (range_cm2 > 0.5) & (range_cm3 > 0.5) & (range_cm4 > 0.5)):            
            message.linear.x = 0.0
            publisher.publish(message)
            self.get_logger().info(f"All Black STOP {message.linear.x}")
            # message.linear.x = -4.0
            # publisher.publish(message)
            # self.get_logger().info(f"Backing Up {message.linear.x}")
            # message.angular.z = 4.0
            # publisher.publish(message)
            # self.get_logger().info(f"Backing Up {message.angular.z}")

        
        elif(range_cm2 > 0.5):            #going straight
            message.linear.x = 4.0
            publisher.publish(message)
            self.get_logger().info(f"Straight {message.linear.x}")
        
        elif(range_cm4 > 0.5):          #far right sensor tripped so turn left
            message.angular.z = 2.0
            publisher.publish(message)
            self.get_logger().info(f"Far Right {message.angular.z}")      

        elif(range_cm0 > 0.5):          #far left sensor tripped so turn right
            message.angular.z = -2.0
            publisher.publish(message)
            self.get_logger().info(f"Far Left {message.angular.z}")

        elif(range_cm3 > 0.5):          #middle right sensor tripped so turn left
            message.angular.z = 1.0
            publisher.publish(message)
            self.get_logger().info(f"Middle Right {message.angular.z}")      

        elif(range_cm1 > 0.5):          #middle left sensor tripped so turn right
            message.angular.z = -1.0
            publisher.publish(message)
            self.get_logger().info(f"Middle Left {message.angular.z}")

        else:
            self.get_logger().info(f"No Line {msg.range}")
            message.linear.x = 0.0
            publisher.publish(message)

        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    
    global node
    node = Node('follower')
    global publisher
    publisher = node.create_publisher(Twist, '/cmd_vel', rclpy.qos.qos_profile_system_default)
   

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)
    rclpy.spin(node)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
