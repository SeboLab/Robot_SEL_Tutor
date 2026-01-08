import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
from mutagen.mp3 import MP3
import time
from openai import OpenAI
from dotenv import load_dotenv
import os
import json
from time import sleep

class SpeechListener(Node):
    def __init__(self):
        super().__init__('speech_listener')
        
        # SUBSCRIPTIONS
        self.subscription = self.create_subscription(String, 'GPTresponse', self.listener_callback, 10)
        
        # PUBLISHERS
        self.status_ = self.create_publisher(String, 'status', 10)
        self.log_ = self.create_publisher(String, 'log', 10)
        self.expression_publisher_ = self.create_publisher(String, 'expression', 10)
        self.initialize_deepgram = self.create_publisher(String, 'deepgram', 10)
        
        # PARAMETERS FROM LAUNCH
        self.declare_parameter('condition', '1')  #default to fictional
        self.declare_parameter('lesson', '1') # default lesson #
        self.condition = self.get_parameter('condition').get_parameter_value().string_value
        self.lesson = self.get_parameter('lesson').get_parameter_value().string_value
        
        # GET ENVRIONMENTAL VARIABLES
        load_dotenv()
        self.misty_ip = os.getenv('MISTY_IP_ADDRESS')
        self.path = os.getenv('FILE_PATH') + "file.mp3"
        if not os.getenv('FILE_PATH'):
            self.get_logger().error("FILE_PATH environment variable not set")
            raise ValueError("FILE_PATH environment variable not set")
        if not self.misty_ip:
            self.get_logger().error("MISTY_IP_ADDRESS environment variable not set")
            raise ValueError("MISTY_IP_ADDRESS environment variable not set")
        self.local_ip = os.getenv('LOCAL_IP_ADDRESS')
        if not self.misty_ip:
            self.get_logger().error("LOCAL_IP_ADDRESS environment variable not set")
            raise ValueError("LOCAL_IP_ADDRESS environment variable not set")
        
        self.first = True
        self.get_logger().info('tts node initialized.')
       
        # delay telling manager the node is initialized just in case the manager has not been initialized
        if self.lesson == '1':
            sleep(2)
            self.publish_to_status("tts_started")
                
    # sends the new expression to the expression topic
    def send_expression(self, string):
        msg = create_msg(string)
        self.expression_publisher_.publish(msg)
    
    # publishes transcripts to log
    def publish_to_log(self, speech):
        data = {}
        data["type"] = "tts"
        data["content"] = speech
        msg = create_msg(json.dumps(data))
        self.log_.publish(msg)
    
    # sends command to start stt when audio done playong
    def publish_to_status(self, msg):
        msg_data = create_msg(msg)
        self.status_.publish(msg_data)
    
    # generates (and plays) audio file and expression from gpt response
    def play_audio(self, msg):
        client = OpenAI()
        response = json.loads(msg)
        audio = client.audio.speech.create(
            model="tts-1",
            voice="alloy",
            input=response["msg"]
        )
        audio.stream_to_file(self.path)
        path = 'http://' + self.local_ip + ':8000/file.mp3'
        requests.post("http://"+self.misty_ip+"/api/audio/play", json={"FileName": path})
        self.send_expression("action " + response["expression"])
        
        self.publish_to_log(response)
        audio = MP3(self.path)
        audio_info = audio.info 
        length = audio_info.length
        # start stt early due to delay in starting streaming
        if (length > 3.5):
            time.sleep(length - 3.5)
            self.send_expression("action body-reset")
            self.publish_to_status("start_stt")
            time.sleep(3.4)
            self.get_logger().info("TTS DONE")
            self.publish_to_status("tts done")
        else:
            self.send_expression("action body-reset")
            self.publish_to_status("start_stt")
            time.sleep(length - 0.1)
            self.get_logger().info("TTS DONE")
            self.publish_to_status("tts done")
    
    # this should only be called if we are on lesson 1
    def perform_intro(self, condition, lesson):
        path = 'http://' + self.local_ip + ':8000/intro_files/'
        self.send_expression("LED 0 0 255")
        self.send_expression("action hi")
        if lesson == '1':
            if condition == '1':
                requests.post("http://"+self.misty_ip+"/api/audio/play", json={"FileName": path+"fictional1.mp3"})
                audio = MP3('./intro_files/fictional1.mp3')
                audio_info = audio.info 
                length = audio_info.length + 0.3 # add here to increase time before next file
                sleep(length)
                requests.post("http://"+self.misty_ip+"/api/audio/play", json={"FileName": path+"fictional2.mp3"})
                audio = MP3('./intro_files/fictional2.mp3')
                audio_info = audio.info 
                length = audio_info.length + 0.3 # add here to increase time before next file
                sleep(length)
                requests.post("http://"+self.misty_ip+"/api/audio/play", json={"FileName": path+"fictional3.mp3"})
                audio = MP3('./intro_files/fictional3.mp3')
                audio_info = audio.info 
                length = audio_info.length + 0.3 # add here to increase time before next file
                sleep(length)
            else:
                requests.post("http://"+self.misty_ip+"/api/audio/play", json={"FileName": path+"factual1.mp3"})
                audio = MP3('./intro_files/factual1.mp3')
                audio_info = audio.info 
                length = audio_info.length + 0.3 # add here to increase time before next file
                sleep(length)
                requests.post("http://"+self.misty_ip+"/api/audio/play", json={"FileName": path+"factual2.mp3"})
                audio = MP3('./intro_files/factual2.mp3')
                audio_info = audio.info 
                length = audio_info.length + 0.3 # add here to increase time before next file
                sleep(length)
                requests.post("http://"+self.misty_ip+"/api/audio/play", json={"FileName": path+"factual3.mp3"})
                audio = MP3('./intro_files/factual3.mp3')
                audio_info = audio.info 
                length = audio_info.length + 0.3 # add here to increase time before next file
                sleep(length)
            self.publish_to_status("start_stt")
            self.send_expression("LED 0 255 0")
            sleep(1)
            requests.post("http://"+self.misty_ip+"/api/audio/play", json={"FileName": path+"intro-2.mp3"})

    def listener_callback(self, msg):
        self.get_logger().info(msg.data)
        if msg.data == "intro":
            self.perform_intro(self.condition, self.lesson)
        else:
            self.play_audio(msg.data)

def create_msg(string):
    ros_msg = String()
    ros_msg.data = string
    return ros_msg

def main(args=None):
    rclpy.init(args=args)

    speech_listener = SpeechListener()

    rclpy.spin(speech_listener)

if __name__ == '__main__':
    main()
