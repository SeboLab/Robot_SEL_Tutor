# node that receives logs from all other nodes
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class WebServer(Node):
    def __init__(self):
        super().__init__('web')
        self.subscription = self.create_subscription(String, 'log', self.listener_callback, 10)
        self.get_logger().info('Manager node initialized.')
    
    # writes logs to file
    # string: string data that was sent through log topic
    def log_to_file(self, string, filename):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        with open(filename, 'a') as file:
            file.write(f'{timestamp} {string}\n')
    
    # msg: String message received from log topic
    def listener_callback(self, msg):
        self.get_logger().info(msg.data)
        self.log_to_file(msg.data, 'log.txt')
        # DEBUGGING: add anywhere else that logs should go

def main(args=None):
    rclpy.init(args=args)
    manager = Manager()
    rclpy.spin(manager)
    
if __name__ == '__main__':
    main()