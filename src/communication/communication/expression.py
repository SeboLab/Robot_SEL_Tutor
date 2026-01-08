'''
ABOUT THE EXPRESSION TOPIC:
Messages through the 'expression' topic are strings.
The string should be prefixed with either 'LED', 'image', or 'action' to 
determine what is being changed or done.
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import os
from dotenv import load_dotenv
import json

# dictionary with json data to remake built-in actions as "{og_name}_temp", but without changing the LED light
actions = {
    "body-reset": {"editable":True,"overwrite":True,"name":"body-reset_temp","script":"IMAGE:e_DefaultContent.jpg; ARMS:89,89,1000; HEAD:-5,0,0,1000;"},
    "mad": {"editable":True,"overwrite":True,"name":"mad_temp","script":"IMAGE:e_DefaultContent.jpg; PAUSE:1000; HEAD:-10,20,15,100; ARMS:89,89,1000; IMAGE:e_Anger.jpg; PAUSE:500; HEAD:-10,-20,-15,100; PAUSE:500; HEAD:-5,0,0,100;"},
    "concerned": {"editable":True,"overwrite":True,"name":"concerned_temp","script":"IMAGE:e_ApprehensionConcerned.jpg; HEAD:-5,-25,0,500; ARMS:-89,-89,2000;"},
    "grief": {"editable":True,"overwrite":True,"name":"grief_temp","script":"IMAGE:e_Grief.jpg; ARMS:89,89,2000; HEAD:10,0,0,3000;"},
    "walk-happy": {"editable":True,"overwrite":True,"name":"walk-happy_temp","script":"IMAGE:e_Joy.jpg; ARMS:-89,89,700; HEAD:-15,-10,-25,700; PAUSE:1100; ARMS:89,-89,700; HEAD:-15,10,25,700; PAUSE:1100; ARMS:-89,89,700; HEAD:-15,-10,-25,700; PAUSE:1100; ARMS:89,-89,700; HEAD:-15,10,25,700;"},
    "listen": {"editable":True,"overwrite":True,"name":"listen_temp","script":"IMAGE:e_Surprise.jpg; HEAD:-6,30,0,1000; PAUSE:2500; HEAD:-5,0,0,500; IMAGE:e_DefaultContent.jpg;"},
    "check-surroundings-slow": {"editable":True,"overwrite":True,"name":"check-surroundings-slow_temp","script":"IMAGE:e_ContentLeft.jpg; HEAD:-5,0,10,500; ARMS:89,89,3000; PAUSE:2000; IMAGE:e_ContentRight.jpg; HEAD:-5,0,-10,500; PAUSE:2000; IMAGE:e_DefaultContent.jpg; HEAD:-5,0,0,500;"},
    "confused": {"editable":True,"overwrite":True,"name":"confused_temp","script":"IMAGE:e_Terror.jpg; HEAD:-5,25,0,500; PAUSE:1500; HEAD:-5,-25,0,500; PAUSE:1500; HEAD:-5,0,0,500; PAUSE:1000; ARMS:-89,-89,2000;"},
    "look-left": {"editable":True,"overwrite":True,"name":"look-left_temp","script":"IMAGE:e_ContentLeft.jpg; HEAD:-5,5,10,500; ARMS:-89,89,700;"},
    "yes": {"editable":True,"overwrite":True,"name":"yes_temp","script":"IMAGE:e_Admiration.jpg; ARMS:-89,89,2000; HEAD:5,0,0,1000; PAUSE:1000; HEAD:-5,0,0,1000;"},
    "walk-slow": {"editable":True,"overwrite":True,"name":"walk-slow_temp","script":"IMAGE:e_ContentLeft.jpg; ARMS:30,89,500; HEAD:-5,-5,15,500; PAUSE:1500; IMAGE:e_ContentRight.jpg; ARMS:89,30,500; HEAD:-5,5,-15,500; PAUSE:1500; IMAGE:e_ContentLeft.jpg; ARMS:30,89,500; HEAD:-5,-5,15,500; PAUSE:1500; IMAGE:e_ContentRight.jpg; ARMS:89,30,500; HEAD:-5,5,-15,500;"},
    "hug": {"editable":True,"overwrite":True,"name":"hug_temp","script":"IMAGE:e_Joy2.jpg; HEAD:-15,0,0,100; ARMS:-89,-89,1000;"},
    "sad3": {"editable":True,"overwrite":True,"name":"sad3_temp","script":"IMAGE:e_Grief.jpg; ARMS:89,89,3000; HEAD:5,0,0,3000;"},
    "worry": {"editable":True,"overwrite":True,"name":"worry_temp","script":"IMAGE:e_ApprehensionConcerned.jpg; ARMS:29,29,1000; HEAD:10,0,0,1000;"},
    "cry-slow": {"editable":True,"overwrite":True,"name":"cry-slow_temp","script":"IMAGE:e_EcstacyHilarious.jpg; HEAD:-5,0,30,200; PAUSE:400; HEAD:-5,25,30,200; PAUSE:400; HEAD:-5,-25,30,200; PAUSE:400; HEAD:-5,25,30,200; PAUSE:400; HEAD:-5,-25,30,200; PAUSE:400; HEAD:-5,0,-30,200; PAUSE:400; HEAD:-5,25,-30,200; PAUSE:400; HEAD:-5,-25,-30,200; PAUSE:400; HEAD:-5,25,-30,200; PAUSE:400; HEAD:-5,-25,-30,200; PAUSE:400; HEAD:-5,0,0,1000; IMAGE:e_DefaultContent.jpg;"},
    "walk-angry": {"editable":True,"overwrite":True,"name":"walk-angry_temp","script":"IMAGE:e_Anger.jpg; HEAD:10,5,25,1000; PAUSE:1000; ARMS:39,-39,500; HEAD:10,-5,-25,1000; PAUSE:1000; ARMS:-39,39,500; HEAD:10,5,25,1000; PAUSE:1000; ARMS:39,-39,500; HEAD:10,-5,-25,1000; PAUSE:1000; ARMS:-39,39,500; HEAD:10,5,25,1000; PAUSE:1000; ARMS:39,-39,500; HEAD:10,-5,-25,1000;"},
    "head-nod-slow": {"editable":True,"overwrite":True,"name":"head-nod-slow_temp","script":"IMAGE:e_DefaultContent.jpg; ARMS:-89,-89,3000; HEAD:5,0,0,500; PAUSE:700; HEAD:-5,0,0,500; PAUSE:700; HEAD:5,0,0,500; PAUSE:700; HEAD:-5,0,0,500; PAUSE:700; HEAD:5,0,0,500; PAUSE:700; HEAD:-5,0,0,500; PAUSE:700; HEAD:5,0,0,500; PAUSE:700; HEAD:-5,0,0,500;"},
    "hi": {"editable":True,"overwrite":True,"name":"hi_temp","script":"IMAGE:e_Joy.jpg; ARMS:-89,89,500;"},
    "look-right": {"editable":True,"overwrite":True,"name":"look-right_temp","script":"IMAGE:e_ContentRight.jpg; HEAD:5,-5,-10,500; ARMS:89,-89,700;"},
    "admire": {"editable":True,"overwrite":True,"name":"admire_temp","script":"IMAGE:e_Admiration.jpg; HEAD:-5,0,0,200; ARMS:-89,89,2000;"},
    "think": {"editable":True,"overwrite":True,"name":"think_temp","script":"IMAGE:e_ContentRight.jpg; HEAD:-15,-10,-10,500; PAUSE:3000; ARMS:89,89,700; HEAD:-5,0,0,700; IMAGE:e_DefaultContent.jpg;"},
    "sad": {"editable":True,"overwrite":True,"name":"sad_temp","script":"IMAGE:e_RemorseShame.jpg; ARMS:89,89,2500; HEAD:-5,0,0,2000;"},
    "terror": {"editable":True,"overwrite":True,"name":"terror_temp","script":"IMAGE:e_Terror.jpg; ARMS:-10,-10,300; HEAD:-5,0,0,300;"},
    "head-up-down-nod": {"editable":True,"overwrite":True,"name":"head-up-down-nod_temp","script":"IMAGE:e_DefaultContent.jpg; HEAD:-15,0,0,500; PAUSE:500; HEAD:5,0,0,500; PAUSE:500; HEAD:-15,0,0,500; PAUSE:500; HEAD:5,0,0,500; PAUSE:500; HEAD:-5,0,0,500; PAUSE:500;"},
    "surprise2": {"editable":True,"overwrite":True,"name":"surprise2_temp","script":"IMAGE:e_Surprise.jpg; HEAD:-10,0,0,300; ARMS:-89,-89,700;"},
    "head-up-arms-up": {"editable":True,"overwrite":True,"name":"head-up-arms-up_temp","script":"IMAGE:e_DefaultContent.jpg; HEAD:-25,0,0,500; ARMS:-90,-90,500; PAUSE:5000;"},
    "head-down-arms-down": {"editable":True,"overwrite":True,"name":"head-down-arms-down_temp","script":"IMAGE:e_DefaultContent.jpg; HEAD:25,0,0,500; ARMS:89,89,500; PAUSE:5000;"}
}

class Expression(Node):
    def __init__(self):
        super().__init__('expression')
        
        # SUBSCRIPTIONS
        self.subscription = self.create_subscription(String, 'expression', self.listener_callback, 10)
        # PUBLISHERS
        self.log_ = self.create_publisher(String, 'log', 10)
        
        # GET ENVIRONMENTAL VARIABLES
        load_dotenv()
        self.misty_ip = os.getenv('MISTY_IP_ADDRESS')
        if not self.misty_ip:
            self.get_logger().error("MISTY_IP_ADDRESS environment variable not set")
            raise ValueError("MISTY_IP_ADDRESS environment variable not set")
    
        self.change_misty_led(0, 0, 255)
        self.create_new_actions()
        self.get_logger().info('Expression node initialized.')
    
    # sends new expression to the log topic
    def send_log(self, string):
        data = {}
        data["type"] = "expression"
        data["content"] = string
        msg = create_msg(json.dumps(data))
        self.log_.publish(msg)
        self.get_logger().info(string)
        
    '''
    Sends a POST request to Misty to change the LED light to the RGB color
    corresponding to the r, g, and b inputs.
    '''
    def change_misty_led(self, r, g, b):
        x = requests.post("http://"+self.misty_ip+"/api/led", json={"Red": r, "Green": g, "Blue": b})
        data = x.json()
        if data['result'] and data['status'] == 'Success':
            string = f"Changed LED light to: {{r: {r}, g: {g}, b: {b}}}"
            self.send_log(string)
    
    '''
    Sends a POST request to Misty to perform one of the built-in actions.
    action_name is the name of the Misty action with spaces replaced by - and 
    all the letters changed to lowercase. For example, if we want to perform
    "Look Right", action_name = "look-right".
    '''
    def misty_action(self, action_name):
        new_name = action_name + '_temp'
        x = requests.post("http://"+self.misty_ip+"/api/actions/start", json={"name": new_name})
        data = x.json()
        if data['result'] and data['status'] == 'Success':
            self.send_log(f"Misty performed action: {{name: {action_name}}}")

    
    '''
    Sends a POST request to Misty to change the image displayed.
    new_image_file_name, with a few exceptions, is usually name of the 
    expression displayed on the Misty dashboard with no spaces, prefixed by "e_"
    and suffixed by ".jpg". For example, Joy -> new_image_file_name = e_Joy.jpg.
    Exceptions include:
        Default -> "e_DefaultContent.jpg"
        Starry Eyes -> "e_EcstacyStarryEyed.jpg"
        Rage 1 -> "e_Rage.jpg"
        Sleeping Zzz -> "e_SleepingZZZ.jpg"
        Camera -> "e_SystemCamera.jpg"
    '''
    def change_misty_image(self, new_image_file_name):
        x = requests.post("http://"+self.misty_ip+"/api/images/display", json={"FileName": new_image_file_name})
        data = x.json()
        if data['result'] and data['status'] == 'Success':
            self.send_log(f"Changed image to: {{FileName: {new_image_file_name}}}")
    
    # msg: String message received from expression topic
    def listener_callback(self, msg):
        msg_data = msg.data
        x = msg_data.split()
        if x[0] == 'LED':
            try:
                self.change_misty_led(int(x[1]), int(x[2]), int(x[3]))
            except:
                self.send_log(create_msg(f"Failed to change LED light with msg: {msg_data}"))
        elif x[0] == 'image':
            try:
                self.change_misty_image(x[1])
            except:
                self.send_log(create_msg(f"Failed to change image with msg: {msg_data}"))
        elif x[0] == 'action':
            try:
                self.misty_action(x[1])
            except:
                self.send_log(f"Failed to do action with msg: {msg_data}")
        else:
            self.send_log(f"Failed to identify what to do from expression msg: {msg_data}")
    
    '''
    Creates new actions based on the specifications above
    '''
    def create_new_actions(self):
        failed_actions = []
        for key, val in actions.items(): 
            x = requests.post("http://"+self.misty_ip+"/api/actions", json=val)
            data = x.json()
            if not data['result'] or data['status'] != 'Success':
                failed_actions.append(key)
        if len(failed_actions):
            self.send_log("Failed to create temporary actions for: " + str(failed_actions))
            self.get_logger().info("Failed to create temporary actions for: " + str(failed_actions))
        else:
            self.get_logger().info("All actions created successfully.")

def create_msg(string):
    ros_msg = String()
    ros_msg.data = string
    return ros_msg
        

def main(args=None):
    rclpy.init(args=args)
    exp = Expression()
    
    rclpy.spin(exp)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    exp.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()