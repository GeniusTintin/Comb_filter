cmake_minimum_required(VERSION 3.0.2)
project(comb)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)

# Declare a C++ library
cs_add_library(${PROJECT_NAME} src/comb.cpp
                               src/bag_player.cpp
                               src/utils.cpp
)

# link required libraries
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})




# Declare a C++ executable
cs_add_executable(${PROJECT_NAME}_node src/comb_node.cpp)


# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

#############
## Install ##
#############

cs_install()