import launch
import launch_ros.actions

def generate_launch_description():
    #asking for input before starting misty to store participant id & condition details
    condition = input("Enter condition ('1' for disclosure & '0' for non-disclosure): ")
    participant = input("Enter subject ID: ")
    lesson = input("Enter lesson number here: ")
    cont = input("Resume previous conversation (enter 0 for no, 1 for yes): ")
    stu_name = input("Student's name: ")

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='communication',
            executable='manager',
            name='manager',
            parameters=[
                {'condition': condition},
                {'participant': participant},
                {'lesson': lesson},
                {'continue': cont}
            ]),
        launch_ros.actions.Node(
            package='communication',
            executable='gpt',
            name='gpt',
            parameters=[
                {'condition': condition},
                {'participant': participant},
                {'lesson': lesson},
                {'continue': cont},
                {'stu_name': stu_name}
            ]),
        launch_ros.actions.Node(
            package='communication',
            executable='tts',
            name='tts',
            parameters=[
                {'condition': condition},
                {'lesson': lesson}
            ]),
        launch_ros.actions.Node(
            package='communication',
            executable='stt',
            name='stt',
            parameters=[
                {'condition': condition},
                {'participant': participant}
            ]),
        launch_ros.actions.Node(
            package='communication',
            executable='expression',
            name='expression'),
    ])
