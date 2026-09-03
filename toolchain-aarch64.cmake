# 交叉编译工具链 —— Radxa 3W (Rockchip RK3566, Cortex-A55)
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 交叉编译器
set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# 目标 ARM 架构，关闭 GNU 扩展避免 ISOC23（Radxa glibc 2.36 不支持 GLIBC_2.38）
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -mcpu=cortex-a55 -march=armv8.2-a")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=cortex-a55 -march=armv8.2-a -std=c++17")
set(CMAKE_CXX_EXTENSIONS OFF)

# ---- sysroot: 从 Radxa 拉取的 OpenCV 4.6 系统库 ----
set(CMAKE_SYSROOT /home/jasonzz/pro/opencv-arm/sysroot-4.6)

# 查找策略：SHARED_LIBRARY 设为 ONLY 强制只用 sysroot 里的 .so
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# 静态链接 libstdc++/libgcc 避免板子上 GLIBCXX 版本不匹配
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libstdc++ -static-libgcc -Wl,--allow-shlib-undefined -Wl,--unresolved-symbols=ignore-in-shared-libs")

# 交叉编译模式
set(CMAKE_CROSSCOMPILING TRUE)
