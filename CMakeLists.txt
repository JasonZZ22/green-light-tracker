cmake_minimum_required(VERSION 3.16)
project(green-light-tracker VERSION 1.0 LANGUAGES CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------- OpenCV (交叉编译时通过 -D 指定 install 路径) ----------
find_package(OpenCV REQUIRED)

# ---------- MAVLink 头文件 ----------
set(MAVLINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/c_library_v2)

# ---------- 源文件 ----------
set(SOURCES
    src/main.cpp
    src/green_detector.cpp
    src/serial_comm.cpp
)

set(HEADERS
    src/green_detector.h
    src/serial_comm.h
)

# ARM 交叉编译兼容桩（解决 GLIBC_2.38 版本不匹配）
if(CMAKE_CROSSCOMPILING)
    list(APPEND SOURCES src/glibc_stub.S)
endif()

# ---------- 可执行文件 ----------
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${MAVLINK_DIR}
)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} pthread)

# ---------- 安装 ----------
install(TARGETS ${PROJECT_NAME} DESTINATION /usr/local/bin)
