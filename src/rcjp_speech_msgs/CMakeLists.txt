cmake_minimum_required(VERSION 2.8.3)
project(rcjp_speech_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs std_srvs message_generation)

# add_message_files(DIRECTORY msg
                      # FILES PanoramaImg.msg
                 #)

add_service_files(DIRECTORY srv
                     FILES SetRecState.srv
                 )

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS std_msgs sensor_msgs std_srvs message_runtime)
