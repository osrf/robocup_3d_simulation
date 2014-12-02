cmake_minimum_required(VERSION 2.8.3)
project(robocup_model_resources)
find_package(catkin)
catkin_package()

# Install the RoboCup worlds.
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install the Nao models.
install(DIRECTORY nao_models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Environment hooks for Gazebo.
set (CMAKE_ROS_PACKAGE_PATH $ENV{ROS_PACKAGE_PATH})
catkin_add_env_hooks(10.gazebo SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
