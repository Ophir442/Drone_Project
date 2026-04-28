cmake_minimum_required(VERSION 3.16)
project(BreadDeliverySimulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find threading library
find_package(Threads REQUIRED)

# Include directories
include_directories(
	${CMAKE_SOURCE_DIR}/include
	${CMAKE_SOURCE_DIR}/third_party
)

# Source files
set(SOURCES
	src/main.cpp
	src/simulation.cpp
	src/distance.cpp
	src/thread_pool.cpp
	src/grasp.cpp
	src/state_logger.cpp
	src/http_server.cpp
)

# Build executable
add_executable(bread_delivery ${SOURCES})

# Link threading
target_link_libraries(bread_delivery Threads::Threads)

# Compiler warnings
target_compile_options(bread_delivery PRIVATE -Wall -Wextra -O2)
