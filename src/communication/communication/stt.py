# speech to text node
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from deepgram import *
import ffmpeg
import os
import requests
from time import sleep
from dotenv import load_dotenv
import json
from datetime import datetime
import time

date = datetime.now().strftime("%m-%d-%Y-%H-%M")

class SpeechToText(Node):
    def __init__(self):
        super().__init__('speech_to_text')
        
        # SUBSCRIPTIONS
        self.status = self.create_subscription(String, 'status', self.status_callback, 10)
        
        # PUBLISHERS
        self.publisher_ = self.create_publisher(String, 'transcript', 10)
        self.status_publisher_ = self.create_publisher(String, 'status', 10)
        self.log_ = self.create_publisher(String, 'log', 10)
        self.expression_publisher_ = self.create_publisher(String, 'expression', 10)
        
        # PARAMETERS FROM LAUNCH
        self.declare_parameter('condition', '1')  #default to disclosure
        self.declare_parameter('participant', 'unknown')  #default participant
        self.condition = self.get_parameter('condition').get_parameter_value().string_value
        self.participant = self.get_parameter('participant').get_parameter_value().string_value
        
        # GET ENVIRONMENTAL VARIABLES
        load_dotenv()
        self.misty_ip = os.getenv('MISTY_IP_ADDRESS')
        if not self.misty_ip:
            self.get_logger().error("MISTY_IP_ADDRESS environment variable not set")
            raise ValueError("MISTY_IP_ADDRESS environment variable not set")
        self.deepgram_api_key = os.getenv('DEEPGRAM_API_KEY')
        if not self.misty_ip:
            self.get_logger().error("DEEPGRAM_API_KEY environment variable not set")
            raise ValueError("DEEPGRAM_API_KEY environment variable not set")
        
        self.is_finals = []
        self.paused = False
        self.cam = False
        self.counter = 1
        self.get_logger().info('stt node initialized.')
        
     # sends command to start stt when audio done playong
    def publish_to_status(self, msg):
        msg_data = create_msg(msg)
        self.status_publisher_.publish(msg_data)
    # sends the new expression to the expression topic
    def send_expression(self, string):
        msg = create_msg(string)
        self.expression_publisher_.publish(msg)

    def send_transcript(self, transcript, start, end):
        self.publish_to_log(transcript, start, end)
        msg = create_msg(transcript)
        self.publisher_.publish(msg)
        
    def publish_to_log(self, speech, start, end):
        data = {}
        data["type"] = "stt"
        data["content"] = speech
        msg = create_msg(json.dumps(data))
        self.log_.publish(msg)
        
    def status_callback(self, msg):
        if msg.data == "start_stt":
            self.paused = False
            self.start_cam()
            # sleep(2)
            self.init_deepgram()

    def start_cam(self):
        self.rtsp_url = 'rtsp://' + self.misty_ip + ':1936'
        stat = requests.get('http://' + self.misty_ip + '/api/services/avstreaming/')
        sleep(.1)
        if (not stat.json()["result"]):
            requests.post('http://' + self.misty_ip + '/api/services/avstreaming/enable')
        sleep(.1)
        requests.post('http://' + self.misty_ip + '/api/avstreaming/stop')
        sleep(.1)
        requests.post('http://' + self.misty_ip + '/api/avstreaming/start', json={"URL": "rtspd:1936","Width": 1920,"Height": 1080,"FrameRate": 30})
        sleep(.5)
        self.cam = True
        self.process = (
                    ffmpeg
                    .input(self.rtsp_url,**{"use_wallclock_as_timestamps": "1", "rtsp_transport": "tcp"})
                )
        # sleep(.05)
        self.start = time.time()



    def init_deepgram(self):
        # Specify the output format (MP3)
        output_format = 'mp3'
        if not os.path.exists('./logs/' + self.participant + '/' + date + '/'):
            os.makedirs('./logs/' + self.participant + '/' + date + '/')
        # Run the FFmpeg command
        self.op1 = self.process.output('-', format="mp3", loglevel="quiet").run_async(pipe_stdout=True, pipe_stdin=True)
        self.op2 = self.process.output('./logs/' + self.participant + '/' + date + '/' + self.condition + '-' + str(self.counter) + '.mp3', format="mp3", loglevel="quiet").run_async(pipe_stdin=True)
        # STEP 1: Create a Deepgram client using the API key
        self.counter += 1
        deepgram = DeepgramClient(self.deepgram_api_key)
        # STEP 2: Create a websocket connection to Deepgram
        dg_connection = deepgram.listen.live.v("1")

        # STEP 3: Define the event handlers for the connection
        def on_message(self_local, result, **kwargs):
            sentence = result.channel.alternatives[0].transcript
            if len(sentence) == 0:
                return
            if result.is_final:
                # We need to collect these and concatenate them together when we get a speech_final=true
                # See docs: https://developers.deepgram.com/docs/understand-endpointing-interim-results
                self.is_finals.append(sentence)
                # Speech Final means we have detected sufficent silence to consider this end of speech
                # Speech final is the lowest latency result as it triggers as soon an the endpointing value has triggered
        def on_end(self_local, **kwargs):
            aud_end = time.time()
            utterance = " ".join(self.is_finals)
            print(f"Speech Final: {utterance}")
            self.send_transcript(utterance, self.aud_start, aud_end)
            self.is_finals = []
            self.paused = True

        def on_error(self_local, error, **kwargs):
            print(f"\n\n{error}\n\n")

        # STEP 4: Register the event handlers
        dg_connection.on(LiveTranscriptionEvents.Transcript, on_message)
        dg_connection.on(LiveTranscriptionEvents.UtteranceEnd, on_end)
        dg_connection.on(LiveTranscriptionEvents.Error, on_error)

        # STEP 5: Configure Deepgram options for live transcription
        options = LiveOptions(
            model="nova-2", 
            language="en-US", 
            smart_format=True,
            interim_results=True,
            utterance_end_ms="2500",
            vad_events=True,
            # Time in milliseconds of silence to wait for before finalizing speech
            endpointing=300
        )
        custom_options: dict = {"mip_opt_out": "true"}

        # STEP 6: Start the connection

        # STEP 8: Define a thread that streams the audio and sends it to Deepgram
        packet_size = 4096
        dg_connection.start(options, addons=custom_options)
        sleep(.2)
        self.publish_to_status("stt done")
        self.aud_start = time.time()
        while not self.paused == True:
            packet = self.op1.stdout.read(packet_size)
            dg_connection.send(packet)
        dg_connection.finish()
        self.op2.communicate(str.encode("q"))
        self.op1.communicate(str.encode("q"))
        sleep(1)
        self.op2.terminate()
        self.op1.terminate()
        requests.post('http://' + self.misty_ip + '/api/avstreaming/stop')
        return;

def create_msg(string):
    ros_msg = String()
    ros_msg.data = string
    return ros_msg

def main(args=None):
    rclpy.init()
    speech_to_text = SpeechToText()
    # speech_to_text.init_deepgram()
    rclpy.spin(speech_to_text)

if __name__ == '__main__':
    main()