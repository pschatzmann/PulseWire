cmake_minimum_required(VERSION 3.14)
project(decoding_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# Declare the external library
FetchContent_Declare(
    arduino_emulator
    GIT_REPOSITORY https://github.com/pschatzmann/Arduino-Emulator
    GIT_TAG main
)

# Make it available
FetchContent_MakeAvailable(arduino_emulator)


# Include the library source directory
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/pulse
)

# Add the test executable
add_executable(decoding_test decoding.cpp)

# Optional: enable warnings
#target_compile_options(decoding_test PRIVATE -Wall -Wextra)
target_compile_definitions(decoding_test PUBLIC HAS_STL)

target_link_libraries(decoding_test arduino_emulator)
