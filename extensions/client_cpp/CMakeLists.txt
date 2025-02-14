cmake_minimum_required(VERSION 3.10)
project(hl2ss)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compile definitions
add_definitions(-DHL2SS_ENABLE_ZDEPTH)

# Find OpenCV package
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ../../3rdparty/Zdepth/include
    ../../3rdparty/Zdepth/zstd/include
    ../../3rdparty/Zdepth/zstd/src
)

# Collect Zdepth source files
file(GLOB ZDEPTH_SOURCES 
    "../../3rdparty/Zdepth/src/*.cpp"
    "../../3rdparty/Zdepth/zstd/src/*.c"
)

# Create executable
add_executable(${PROJECT_NAME}
    main.cpp
    hl2ss.cpp
    hl2ss_lnm.cpp
    hl2ss_mt.cpp
    ${ZDEPTH_SOURCES}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    avcodec
    avutil
    pthread
)

# Add library search path
target_link_directories(${PROJECT_NAME} PRIVATE /usr/local/lib)

# Set compiler flags
target_compile_options(${PROJECT_NAME} PRIVATE 
    -O3
    -fdiagnostics-color=always
    -g
)