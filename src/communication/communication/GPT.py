# in order to work, first you will need to create an API through: https://aistudio.google.com/app/apikey
# then, put the API key in the .env file
# import necessary libraries (if you don't have them, pip install)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import google.generativeai as genai
from google.generativeai.types import HarmCategory, HarmBlockThreshold, generation_types
from IPython.display import Markdown
import textwrap
import os
import json
from time import sleep
from dotenv import load_dotenv

#function to print text in a nice format
def to_markdown(text):
    text = text.replace('•', '  *')
    return Markdown(textwrap.indent(text, '> ', predicate=lambda _: True))   

class GPT(Node):
    def __init__(self):
        super().__init__('GPT')
        
        # SUBSCRIPTIONS
        self.subscription = self.create_subscription(String, 'transcript', self.listener_callback, 10)
        
        # PUBLISHERS
        self.response_publisher_ = self.create_publisher(String, 'GPTresponse', 10)
        self.expression_publisher_ = self.create_publisher(String, 'expression', 10)
        self.log_ = self.create_publisher(String, 'log', 10)
        
        # PARAMETERS FROM LAUNCH
        self.declare_parameter('condition', '1')  #default to disclosure
        self.declare_parameter('participant', 'unknown')  #default participant
        self.declare_parameter('continue', '0')  #default to new conversation
        self.declare_parameter('stu_name', 'unknown')  #default name
        self.declare_parameter('lesson', '1') # default lesson #
        self.condition = self.get_parameter('condition').get_parameter_value().string_value
        self.participant = self.get_parameter('participant').get_parameter_value().string_value
        self.resume = self.get_parameter('continue').get_parameter_value().string_value
        self.name = self.get_parameter('stu_name').get_parameter_value().string_value
        self.lesson = self.get_parameter('lesson').get_parameter_value().string_value

        # GET ENVIRONMENTAL VARIABLES
        load_dotenv()
        api_key = os.getenv('GOOGLE_API_KEY')
        if not api_key:
            self.get_logger().error("GOOGLE_API_KEY environment variable not set")
            raise ValueError("GOOGLE_API_KEY environment variable not set")
        
        genai.configure(api_key=api_key)
        prompt = generate_prompt(self.condition, self.name, self.lesson, self.participant)

        #starting the chat with the Gemini model
        self.model = genai.GenerativeModel(
            model_name='gemini-1.5-pro',
            system_instruction=prompt, 
            safety_settings={
                HarmCategory.HARM_CATEGORY_HARASSMENT: HarmBlockThreshold.BLOCK_LOW_AND_ABOVE,
                HarmCategory.HARM_CATEGORY_DANGEROUS_CONTENT: HarmBlockThreshold.BLOCK_LOW_AND_ABOVE,
                HarmCategory.HARM_CATEGORY_HATE_SPEECH: HarmBlockThreshold.BLOCK_LOW_AND_ABOVE,
                HarmCategory.HARM_CATEGORY_SEXUALLY_EXPLICIT: HarmBlockThreshold.BLOCK_LOW_AND_ABOVE,
            }, 
            generation_config={"temperature": 0, "response_mime_type": "application/json"}
        )
        if self.resume == '0':
            if self.lesson == '1':
                if self.condition == '1':
                    d_or_nd = 'disclosure'
                else:
                    d_or_nd = 'nondisclosure'
                with open(f'intro_histories/lesson_{self.lesson}_{d_or_nd}_intro.txt', 'r') as file:
                    msg = file.read()
                history = [{
                    'role': 'model',
                    'parts': [{"text":f"""{{
                        "msg": "{msg}",
                        "expression": "hi",
                        "stage": "Introduce yourself"}}"""}]
                }]
            else:
                history = []
        else:
            history = []
            filename = f"./logs/{self.participant}_lesson{self.lesson}.json"
            with open(filename, 'r') as file:
                data = json.load(file)["convos"]
                msgs = data[len(data)-1]["messages"]
                for i in range(0, len(msgs)):
                    if msgs[i]["role"] == "stt":
                        role = "user"
                    else:
                        role = "model"
                    if not (role == "user" and i == len(msgs)-1):
                        history.append({
                            'role': role,
                            'parts': [{'text': json.dumps(msgs[i]["content"])}]
                        })
                file.close()
        self.chat = self.model.start_chat(history=history)
        self.get_logger().info('GPT node initialized.')
        
        if self.lesson != '1':
            sleep(3)
            self.send_msg('Start conversation')
            self.get_logger().info('GPT started conversation.')
        
    # sends the GPT response to the GPTresponse topic
    def send_transcript(self, response):
        msg = create_msg(response)
        self.response_publisher_.publish(msg)
    
    # sends the new expression to the expression topic
    def send_expression(self, string):
        msg = create_msg(string)
        self.expression_publisher_.publish(msg)

    # sends GPT response to the log topic
    def send_log(self, log):
        self.log_.publish(log)
        self.get_logger().info(log.data)

    def send_msg(self, msg):
        ### CHANGE # OF TRIES
        tries = 3
        cont = 1
        # try to come up with a response
        while tries and cont:
            try:
                tries -= 1
                response = self.chat.send_message(msg)
                cont = 0
            except generation_types.StopCandidateException as E:
                self.get_logger().info("dangerous content")
                self.get_logger().info(str(E))
            except Exception as E:
                self.get_logger().info("other exception")
                self.get_logger().info(str(E))
        # tries to cut to the next question
        if cont:
            tries = 3
            cont = 1
            while tries and cont:
                try:
                    tries -= 1
                    response = self.chat.send_message("Please move on to the next question.")
                    cont = 0
                except Exception as E:
                    self.get_logger().info("other exception while attempting to move on")
                    self.get_logger().info(str(E))
        # if the LLM reallyyyyy can't recover
        if cont:
            response = '''{"msg": "Hm. I didn't quite catch that. Let's move on.","expression": "body-reset","stage": "Resolve Issue"}'''
            self.send_transcript(response)
            history = []
            filename = f"./logs/{self.participant}_lesson{self.lesson}.json"
            with open(filename, 'r') as file:
                data = json.load(file)["convos"]
                msgs = data[len(data)-1]["messages"]
                for i in range(0, len(msgs)):
                    if msgs[i]["role"] == "stt":
                        role = "user"
                    else:
                        role = "model"
                    if not (role == "user" and i == len(msgs)-1):
                        history.append({
                            'role': role,
                            'parts': [{'text': json.dumps(msgs[i]["content"])}]
                        })
                file.close()
            self.chat = self.model.start_chat(history=history)
        elif response:
            self.get_logger().info(f"Response: {response.text}")
            self.send_transcript(response.text)
        else:
            self.get_logger().error("No response received from Gemini")

    # msg: String message received from transcript topic, i.e. transcript of the student's speech
    def listener_callback(self, msg):
        self.send_expression("LED 0 0 255")
        self.get_logger().info(msg.data)
        self.send_msg(msg.data)
        
def create_msg(string):
    ros_msg = String()
    ros_msg.data = string
    return ros_msg

def generate_prompt(condition, name, lesson, participant):
    if lesson != "1":
        prev_lesson = str(int(lesson) - 1)
        with open(f'logs/{participant}_lesson{prev_lesson}_summary.txt') as f:
            previous_conversation = f.read()
        f.close()
    if condition == "1":
        with open('prompts/lesson_' + lesson + '_disclosure_prompt.txt', 'r') as file:
            prompt = file.read()
        file.close()
    else:
        with open('prompts/lesson_' + lesson + '_nondisclosure_prompt.txt', 'r') as file:
            prompt = file.read()
        file.close()
    prompt = prompt.replace("[[name]]", name)
    if lesson != "1":
        prompt = prompt.replace("[[previous_conversation]]", previous_conversation)
    return prompt

def main(args=None):
    rclpy.init(args=args)
    gpt_node = GPT()
    rclpy.spin(gpt_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gpt_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
