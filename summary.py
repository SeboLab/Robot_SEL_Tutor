import google.generativeai as genai
from google.generativeai.types import HarmCategory, HarmBlockThreshold, generation_types
from IPython.display import Markdown
import os
import json
from dotenv import load_dotenv

class Summary():
    def __init__(self, subj, name, file, lesson):
        load_dotenv()
        api_key = os.getenv('GOOGLE_API_KEY')
        if not api_key:
            self.get_logger().error("GOOGLE_API_KEY environment variable not set")
            raise ValueError("GOOGLE_API_KEY environment variable not set")

        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel(
            model_name='gemini-1.5-pro',
            generation_config={"temperature": 0}
        )
        self.chat = self.model.start_chat()
        
        transcript = load_transcript(file, subj, name)

        response = self.chat.send_message(f'''Please write a summary of the following conversation between a robot named Misty and a student named {name}. 
                                          There should be a summary of the conversation as a whole and a summary of what {name} disclosed: {transcript}''')
        
        with open(f'logs/{subj}_lesson{lesson}_summary.txt', 'w') as f:
            f.write(response.text)

def load_transcript(file, subj, name):
    name = name.upper()
    f = open(file)
    data = json.load(f)
    transcript = []
    for section in data['convos']:
        for x in section['messages']:
            if x['role'] == 'stt':
                user_said = f'{name}: {x['content']}'
                transcript.append(user_said)
            else:
                robot_said = f'MISTY: {x['content']['msg']}'
                transcript.append(robot_said)
    return '\n'.join(transcript)

if __name__ == "__main__":
    subj = input("Subject ID: \n")
    name = input("Student's name: \n")
    lesson_num = input("Lesson number: \n")
    file = f"logs/{subj}_lesson{lesson_num}.json"
    Summary(subj, name, file, lesson_num)
    