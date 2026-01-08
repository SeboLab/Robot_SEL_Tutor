# Misty SEL Project Development
## What is this?
This is a set of ROS2 python scripts to control the Misty II robot for the SEL HRI project.

## MacOS Users
In order to install ROS2, we have found that it is easier to create a virtual linux machine and install it there. Virtual machine options that we have found to work include:

### Multipass (worked with Intel Core)
1. Install Multipass by following [these instructions](https://multipass.run/docs/install-multipass)
2. Follow [these instructions](https://multipass.run/docs/create-an-instance#create-an-instance) to make an instance.
3. Stop your instance by running `multipass stop [your instance's name]` in your terminal.
4. Give your instance a larger disk size using `multipass set local.[your instance's name].disk=20G`
5. Start the instance again `multipass start [your instance's name]`
6. Get your instance's IP address by running `multipass info`, which you can use to set up SSH access in VSCode.
7. In order to play audio files on Misty, you will need to mount the direcotry where the audio files are being saved to a local direcotry.
    a. Instructions to do so via command line: [(instructions here)](https://multipass.run/docs/mount-command).
    b. You can also do this on the Multipass app by going to your instance, then to Details, and scrolling down to Mounts. The source path should be a directory on your local machine (do not make this directory in your Documents or Downloads folders, as they have different read/write permissions), and the target path should be a directory in your virtual machine where the audio files will be saved (i.e. the path specified in `FILE_PATH` in the `.env` file).
8. Download the intro_files folder and move it into your mounted directory. 

### VirtualBox (worked with Intel Core)
1. Install VirtualBox using [these]('https://www.virtualbox.org/wiki/Downloads') instructions.
2. Download Ubuntu 24.04 (or whichever Ubuntu version ROS2 Jazzy says its compatable with [here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)) (https://ubuntu.com/download/desktop)
3. Create a new virtual machine using the ISO and correct Linux version. You likely want to allocate at least 8GB of memory, 4 CPU cores, and 25GB of storage.
4. Follow the steps to install Ubuntu. You may need to attach an ISO as a CD if it does not do so automatically.

### UTM (worked with M1 chip)
1. Install UTM for free: https://mac.getutm.app/ (note: if you try to download from app store directly, you will be required to pay $9.99)
2. Install Ubuntu: https://docs.getutm.app/guides/ubuntu/
3. Install ROS2 (instructions below)
4. Clone repo (to setup ssh-key: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)

## Setup

### 1. Install ROS2 Jazzy Jalisco on your machine
- Instructions can be found [here](https://docs.ros.org/en/jazzy/Installation.html).
- If you are using Ubuntu (whether through a VM or not), use the [Ubuntu (Debian packages)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html) install.

### 2. Clone this repository and navigate into it.

### 3. Install Python dependencies. These haven't been put into a requirements.txt file yet, but they are:
- openai
- requests (this might come with your Python installation)
- deepgram-sdk
- python-dotenv
- google-generativeai \
To install these, use pip: `python -m pip install [package]` or `python3 -m pip install [package]` depending on your setup. Please note if you use a venv, you may run into problems with ROS not finding the dependencies, so it is recommended that you install these user or systemwide. 

### 4. Create a file called .env with the following lines:
```
GOOGLE_API_KEY=[your Gemini API key]
DEEPGRAM_API_KEY=[your Deepgram API key]
OPENAI_API_KEY=[your OpenAI API key]
MISTY_IP_ADDRESS=[Misty's IP address]
LOCAL_IP_ADDRESS=[your local IP address]
FILE_PATH=[where you want the audio files to be stored (ex. if you wanted to have it be saved in the current directory, you could set this to ./). They will always be named file.mp3. If you are using a mounted directory, set the path to that mounted directory.]
```

### 5. Create a folder called "logs"
You need to create this folder in your working directory to ensure that your transcript and audio files are saved. 
To do so, you can simply type `mkdir logs` in your terminal.

## Running a lesson

### 1. Build the code 
  `colcon build`

### 2. Source ROS environment (in a seperate terminal from the one used to build the code) 
  `source install/setup.bash`

### 3. Start the Python web server
  `python3 -m http.server` \
  This is necessary because the file is stored on your local machine (at the file path set above), but needs to be sent to Misty. Run this in the directory of where the audio file is stored.

### 4. Start the nodes 
  `ros2 launch communication misty_launch.py`
  If this is any lesson past lesson 1, make sure that a summary of the previous lesson transcript has been generated.

## Generating a summary of a conversation
1. Navigate to the root directory. 
2. Run `python3 summary.py`. 
3. You will be prompted to input the student's name and lesson number, which will be used to identify the log to create the summary off of. 
4. The summary will be stored at: `logs/{name}_lesson{number}_summary.txt`.

## Other Tips
- Sometimes when Misty is plugged in to the charger, the LED light will quickly flash blue/green, then go back to orange. Unplugging it should fix this.
- If the process does not stop when you type `ctrl-c`, in another terminal, run `pkill '^ros2$'`

## TODO in the Fall
- come up with new lesson plans
    - make sure new prompts implements a summary of the previous conversation
    - think about ways to enforce the condition in the introduction of the new lesson
- implement pause/resume streaming via button push