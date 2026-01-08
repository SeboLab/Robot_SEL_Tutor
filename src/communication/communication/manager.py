# node that receives logs from all other nodes
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime
import json
from time import sleep

class Manager(Node):
    def __init__(self):
        super().__init__('manager')
        
        # SUBSCRIPTIONS
        self.subscription = self.create_subscription(String, 'log', self.listener_callback, 10)
        self.status_sub = self.create_subscription(String, 'status', self.status_callback, 10)
        
        # PUBLISHERS
        self.publisher = self.create_publisher(String, 'GPTresponse', 10)
        self.expression_publisher_ = self.create_publisher(String, 'expression', 10)
        self.transcript_publisher_ = self.create_publisher(String, 'transcript', 10)

        # PARAMETERS FROM LAUNCH
        self.declare_parameter('condition', '1')  #default to disclosure
        self.declare_parameter('participant', 'unknown')  #default participant
        self.declare_parameter('continue', '0')  #default participant
        self.declare_parameter('lesson', '1')  #default lesson #
        condition = self.get_parameter('condition').get_parameter_value().string_value
        participant = self.get_parameter('participant').get_parameter_value().string_value
        resume = self.get_parameter('continue').get_parameter_value().string_value
        lesson = self.get_parameter('lesson').get_parameter_value().string_value
        self.lesson = lesson
        
        self.stt_done = False
        self.tts_done = False
        self.filename = "./logs/" + participant + '_lesson' + lesson + ".json"
        self.logtxtname = "./logs/" + participant + '_lesson' + lesson + ".txt"
        data = {}
        data["content"] = f"Participant: {participant} | Condition: {condition}"
        data["type"] = "manager"
        self.log_to_file(json.dumps(data))
        
        # starting new convo vs resuming
        try:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            # create if not exist
            with open(self.filename, 'a') as file:
                file.close()
            with open(self.filename, 'r') as file:
                try:
                    data = json.load(file)
                    file.close()
                except ValueError as err:
                    data = json.loads('{"convos": []}')
                    file.close()
            if resume == '0':
                if lesson == '1':
                    if condition == '1':
                        data["convos"].append({"condition": condition, "start": timestamp, "messages": [{"content": {"msg": "Hi! My name is Misty, and I'm a robot who works at a research lab at anonymized. I really enjoy reading all kinds of books, solving puzzles, and spending time with my robot friends. At the lab, I work with my best friends Now and Vector, who have taught me a lot about friendship. We love to share stories and learn new things together. My favorite thing to do is to help people understand each other better! I'm excited to talk to you about problem solving today! But first, I need to tell you about how to interact with me. Sometimes I have trouble hearing, so when you talk to me, you need to make sure my light is green. When it's blue, that means I'm thinking or speaking. Now, my light is green so you can talk to me. What's your name?", "expression": "hi", "stage": "Introduce Yourself"}, "role": "tts"}]})
                    else:
                        data["convos"].append({"condition": condition, "start": timestamp, "messages": [{"content": {"msg": "Hi! My name is Misty, and I'm a robot from a research lab at anonymized. Even though I don't feel emotions myself, I've learned a lot about them through the internet, reading a lot of books, and listening to people chat. My brain is kind of like a library full of words and ideas. When you talk to me, I mix all the words and ideas I know and put them together like a puzzle to come up with responses! My programmers have given me a lesson about problem solving to do with you today! But first, I need to tell you about how to interact with me. My microphone isn't always on, so when you talk to me, you need to make sure my light is green. When it's blue, that means I'm thinking or speaking. Now, my light is green so you can talk to me. What's your name?", "expression": "hi", "stage": "Introduce Yourself"}, "role": "tts"}]})
                    with open (self.filename, 'w') as file:
                        json.dump(data, file)
                        file.close()
                    self.msg = "intro"
                else:
                    data["convos"].append({"condition": condition, "start": timestamp, "messages": []})
                    with open (self.filename, 'w') as file:
                        json.dump(data, file)
                        file.close()
            else:
                messages = data["convos"][len(data["convos"])-1]["messages"]
                for i in range(0, len(messages)):
                    if messages[len(messages)-i-1]["role"] == "tts":
                        message = json.dumps(messages[len(messages)-i-1]["content"])
                        self.msg = message
                        break
        except Exception as e:
            self.get_logger().error(f"Failed to get parameters: {e}")
            
        self.get_logger().info('Manager node initialized.')
        
    # sends the new expression to the expression topic
    def send_expression(self, string):
        msg = create_msg(string)
        self.expression_publisher_.publish(msg)

    def log_to_file(self, json_str):
        msg = json.loads(json_str)
        timestamp = datetime.now().strftime('%m-%d-%Y %H:%M:%S')
        if msg["type"] == "stt" or msg["type"] == "tts":
            with open(self.filename, 'r') as file:
                data = json.load(file)
                file.close()
            data["convos"][len(data["convos"])-1]["messages"].append({"content": msg["content"], "role": msg["type"], "timestamp": timestamp})
            with open(self.filename, 'w') as file:
                json.dump(data, file)
                file.close()
        # log.txt is a log of everything done by every node
        with open(self.logtxtname, 'a') as file:
            file.write(f'{timestamp} | {msg["type"]}: {msg["content"]}\n')
            file.close()

    def listener_callback(self, msg):
        # self.get_logger().info(msg.data)
        self.log_to_file(msg.data)
        
    def status_callback(self, msg):
        if msg.data == "tts_started":
            if self.lesson == '1':
                msg = create_msg(self.msg)
                self.publisher.publish(msg)
        elif msg.data == "tts done":
            self.tts_done = True
            if self.tts_done and self.stt_done:
                self.send_expression("LED 0 255 0")
                self.tts_done = False
                self.stt_done = False
        elif msg.data == "stt done":
            self.stt_done = True
            if self.tts_done and self.stt_done:
                self.send_expression("LED 0 255 0")
                self.tts_done = False
                self.stt_done = False

def create_msg(string):
    ros_msg = String()
    ros_msg.data = string
    return ros_msg

def main(args=None):
    rclpy.init(args=args)
    manager = Manager()
    rclpy.spin(manager)

if __name__ == '__main__':
    main()