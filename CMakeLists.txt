cmake_minimum_required(VERSION 3.10)
project(YKS_SDK)

# Keep compatibility with module-mode Boost lookup on newer CMake.
if(POLICY CMP0167)
        cmake_policy(SET CMP0167 OLD)
endif()

if (NOT TARGET CycloneDDS-CXX::ddscxx)
    find_package(CycloneDDS-CXX REQUIRED)
endif ()
# 查找并添加第三方库目录
add_subdirectory(app/thirdparty)

# 查找 Boost 和 PkgConfig 包
find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)

# 查找 ncurses 库
pkg_check_modules(NCURSES REQUIRED ncurses)

# 设置源文件
set(SOURCES
        app/EYOUControl.cpp
        app/EYOUControl.h
        app/motor_control.c
        app/motor_control.h
        app/math_ops.c
        app/math_ops.h
        app/transmit.cpp
        app/transmit.h
        app/config.h
        app/queue.h
        app/command.cpp
        app/command.h
)

# 创建 motor 库
add_library(motor ${SOURCES})
target_include_directories(motor PRIVATE
        app/thirdparty/soem/soem
        app/thirdparty/soem/osal
        app/thirdparty/soem/osal/linux
        app/thirdparty/soem/oshw/linux
        app/thirdparty/cpp-readline/src
)
target_link_libraries(motor PRIVATE soem)

# 强制使用 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # 如果你不想使用 GNU 扩展，可以将此选项设为 OFF

# 这个是调用生成器转换idl消息的，它自动能处理nubotmsg.hpp的依赖
# 选择手动转换idl消息，则需要处理较为复杂的依赖关系，因此直接使用生成器更为方便
idlcxx_generate(TARGET nubotddsmsg FILES nubotddsmsg.idl)

# 添加可执行文件 YKS_SDK
add_executable(YKS_SDK main.cpp JoyStickHandler.cpp Z1Legs.cpp BmsHandler.cpp SBusReceiver.cpp SocketSender.cpp SocketReceiver.cpp
        DDSRelate.cpp
        DDSRelate.h
        MotorDataLogger.h
        MotorDataLogger.cpp)

# 为 YKS_SDK 添加头文件搜索路径
target_include_directories(YKS_SDK PRIVATE
        ${Boost_INCLUDE_DIRS}
        ${NCURSES_INCLUDE_DIRS}
        app/thirdparty/soem/soem
        app/thirdparty/soem/osal
        app/thirdparty/soem/osal/linux
        app/thirdparty/soem/oshw/linux
        app/thirdparty/cpp-readline/src
)

# 链接所有需要的库到 YKS_SDK
target_link_libraries(YKS_SDK PRIVATE
        motor
        soem
        cpp-readline
        ${NCURSES_LIBRARIES}
        CycloneDDS-CXX::ddscxx nubotddsmsg
)
# Disable the static analyzer in GCC to avoid crashing the GNU C++ compiler
# on Azure Pipelines
if (DEFINED ENV{SYSTEM_TEAMFOUNDATIONSERVERURI})
    if (CMAKE_C_COMPILER_ID STREQUAL "GNU" AND ANALYZER STREQUAL "on")
        target_compile_options(cxxpub PRIVATE -fno-analyzer)
        target_compile_options(cxxsub PRIVATE -fno-analyzer)
    endif ()
endif ()