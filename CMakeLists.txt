cmake_minimum_required(VERSION 3.2)
project(inner_rect)

add_compile_options(-g -std=c++11)

find_package(OpenCV)

FILE(GLOB SRC_FILE
	${CMAKE_SOURCE_DIR}/*.cpp
)

add_executable(${PROJECT_NAME}
	${SRC_FILE}
)

target_link_libraries(${PROJECT_NAME}
	${OpenCV_LIBRARIES}	
)