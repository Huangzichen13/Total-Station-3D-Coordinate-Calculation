cmake_minimum_required(VERSION 3.5)
project(TotalStationCalculator)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找Eigen3库
find_package(Eigen3 REQUIRED)

# 添加可执行文件
add_executable(Get3 src/Get3.cpp)
add_executable(Get6 src/Get6.cpp)

# 链接Eigen3库（两个都需要链接）
target_link_libraries(Get3 Eigen3::Eigen)
target_link_libraries(Get6 Eigen3::Eigen)

# 设置编译选项
target_compile_options(Get3 PRIVATE -Wall -Wextra -O2)
target_compile_options(Get6 PRIVATE -Wall -Wextra -O2)