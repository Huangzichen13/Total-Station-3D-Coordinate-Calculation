cmake_minimum_required(VERSION 3.5)
project(TotalStationCalculator)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找Eigen3库
find_package(Eigen3 REQUIRED)

# 查找OpenCV库（用于PNP程序）
find_package(OpenCV REQUIRED)

# 添加可执行文件 - 全站仪计算程序
add_executable(Get3 src/Get3.cpp)
add_executable(Get6 src/Get6.cpp)

# 添加可执行文件 - PnP相机位姿估计程序
add_executable(PNP src/PNP.cpp)

# 链接Eigen3库
target_link_libraries(Get3 Eigen3::Eigen)
target_link_libraries(Get6 Eigen3::Eigen)

# 链接OpenCV和Eigen3库给PNP
target_link_libraries(PNP ${OpenCV_LIBS} Eigen3::Eigen)

# 设置编译选项 - Get3
target_compile_options(Get3 PRIVATE -Wall -Wextra -O2)

# 设置编译选项 - Get6
target_compile_options(Get6 PRIVATE -Wall -Wextra -O2)

# 设置编译选项 - PNP
target_compile_options(PNP PRIVATE -Wall -Wextra -O2)

# 可选：设置PNP包含目录（如果需要额外路径）
target_include_directories(PNP PRIVATE ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
