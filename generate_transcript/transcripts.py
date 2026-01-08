import logging
import os
import json

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

def process_file(file):
    f = open(file)
    data = json.load(f)
    
    filename = os.path.splitext(file)[0]
    output_file = f"{filename}_raw_transcript.txt"
    
    with open(output_file, "w") as out_file:
        for x in data['convos'][-1]['messages']:
            if x['role'] == 'stt':
                out_file.write('USER: ' + x['content'] + '\n')
            else:
                out_file.write("Misty: " + x['content']['msg'] + '\n')
    
    logger.info(f"Finished {output_file}")
            
if __name__ == "__main__":
    files = [f for f in os.listdir() if f.endswith(".json")]
    logger.info(f"Found files: {files}")
    
    for file in files:
        process_file(file)