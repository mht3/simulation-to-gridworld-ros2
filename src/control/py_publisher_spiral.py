import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import sleep
import numpy as np
import sys

class TurtleBot(Node):

    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.node = super().__init__('py_topic_publisher_' + robot_name)
        print(self.node)
        self.publisher = self.create_publisher(Twist, "/" + robot_name + '/cmd_vel', 1)

    def forward(self):
        message = Twist()
        message.linear.x = 1.0
        self.get_logger().info('Sending - Linear Velocity : %f, Angular Velocity : %f' % (message.linear.x, message.angular.z))
        self.publisher.publish(message)
        # self.i += float(sys.argv[3])
        sleep(1)
        self.publisher.publish(Twist())
        sleep(1)
    

    def backward(self):
        message = Twist()
        message.linear.x = -1.0
        self.get_logger().info('Sending - Linear Velocity : %f, Angular Velocity : %f' % (message.linear.x, message.angular.z))
        self.publisher.publish(message)
        # self.i += float(sys.argv[3])
        sleep(1)
        self.publisher.publish(Twist())
        sleep(1)

    def left(self):
        message = Twist()
        message.angular.z = np.pi/2
        self.get_logger().info('Sending - Linear Velocity : %f, Angular Velocity : %f' % (message.linear.x, message.angular.z))
        self.publisher.publish(message)
        # self.i += float(sys.argv[3])
        sleep(1)
        self.publisher.publish(Twist())
        sleep(1.5)
        self.forward()
        sleep(1)        
    
    def right(self):
        message = Twist()
        message.angular.z = -np.pi/2
        self.get_logger().info('Sending - Linear Velocity : %f, Angular Velocity : %f' % (message.linear.x, message.angular.z))
        self.publisher.publish(message)
        # self.i += float(sys.argv[3])
        sleep(1)
        self.publisher.publish(Twist())
        sleep(1.5)
        self.forward()
        sleep(1)

    def move(self, action: int):
        if action == 1:
            self.forward()
        if action == 2:
            self.backward()
        
def main(args=None):
    rclpy.init(args=None)
    minimal_publisher = TurtleBot(sys.argv[1])
    # rclpy.spin(minimal_publisher)
    minimal_publisher.forward()
    minimal_publisher.left()
    minimal_publisher.right()
    minimal_publisher.backward()
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()