cmake_minimum_required(VERSION 2.8)
project(nao_meshes)

find_package(catkin)
catkin_package()

install(DIRECTORY nao_meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
