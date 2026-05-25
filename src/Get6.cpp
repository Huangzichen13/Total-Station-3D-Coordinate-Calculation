#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>

class PoseEstimator {
public:
    struct Point3D {
        double x, y, z;
        Point3D() : x(0), y(0), z(0) {}
        Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    };
    
    struct Pose6D {
        Eigen::Vector3d translation;
        Eigen::Matrix3d rotation;
        double roll, pitch, yaw;
        
        void print() const {
            std::cout << std::fixed << std::setprecision(6);
            std::cout << "\n========================================" << std::endl;
            std::cout << "    六自由度位姿计算结果" << std::endl;
            std::cout << "========================================" << std::endl;
            
            std::cout << "\n【平移向量 Translation】" << std::endl;
            std::cout << "  X: " << translation.x() << " 米" << std::endl;
            std::cout << "  Y: " << translation.y() << " 米" << std::endl;
            std::cout << "  Z: " << translation.z() << " 米" << std::endl;
            
            std::cout << "\n【旋转矩阵 Rotation Matrix】" << std::endl;
            std::cout << "  " << rotation(0,0) << "  " << rotation(0,1) << "  " << rotation(0,2) << std::endl;
            std::cout << "  " << rotation(1,0) << "  " << rotation(1,1) << "  " << rotation(1,2) << std::endl;
            std::cout << "  " << rotation(2,0) << "  " << rotation(2,1) << "  " << rotation(2,2) << std::endl;
            
            std::cout << "\n【欧拉角 Euler Angles (度数)】" << std::endl;
            std::cout << "  横滚角 Roll:  " << roll * 180 / M_PI << "°" << std::endl;
            std::cout << "  俯仰角 Pitch: " << pitch * 180 / M_PI << "°" << std::endl;
            std::cout << "  偏航角 Yaw:   " << yaw * 180 / M_PI << "°" << std::endl;
            
            std::cout << "\n【齐次变换矩阵 Homogeneous Transform】" << std::endl;
            std::cout << "  " << rotation(0,0) << "  " << rotation(0,1) << "  " << rotation(0,2) << "  " << translation.x() << std::endl;
            std::cout << "  " << rotation(1,0) << "  " << rotation(1,1) << "  " << rotation(1,2) << "  " << translation.y() << std::endl;
            std::cout << "  " << rotation(2,0) << "  " << rotation(2,1) << "  " << rotation(2,2) << "  " << translation.z() << std::endl;
            std::cout << "  0.000000  0.000000  0.000000  1.000000" << std::endl;
            std::cout << "========================================" << std::endl;
        }
    };
    
    static Pose6D computePose(const std::vector<Point3D>& world_points, 
                              const std::vector<Point3D>& local_points) {
        if (world_points.size() < 3 || local_points.size() < 3) {
            throw std::runtime_error("错误：至少需要3个点来计算位姿");
        }
        return computePoseByThreePoints(world_points, local_points);
    }
    
private:
    static Pose6D computePoseByThreePoints(const std::vector<Point3D>& world_points,
                                           const std::vector<Point3D>& local_points) {
        Eigen::Vector3d p1_w(world_points[0].x, world_points[0].y, world_points[0].z);
        Eigen::Vector3d p2_w(world_points[1].x, world_points[1].y, world_points[1].z);
        Eigen::Vector3d p3_w(world_points[2].x, world_points[2].y, world_points[2].z);
        
        Eigen::Vector3d q1_l(local_points[0].x, local_points[0].y, local_points[0].z);
        Eigen::Vector3d q2_l(local_points[1].x, local_points[1].y, local_points[1].z);
        Eigen::Vector3d q3_l(local_points[2].x, local_points[2].y, local_points[2].z);
        
        // 计算局部坐标系中的基向量
        Eigen::Vector3d u = (q2_l - q1_l).normalized();
        Eigen::Vector3d v_temp = (q3_l - q1_l);
        Eigen::Vector3d v = (v_temp - v_temp.dot(u) * u).normalized();
        Eigen::Vector3d w = u.cross(v);
        
        Eigen::Matrix3d R_local;
        R_local.col(0) = u;
        R_local.col(1) = v;
        R_local.col(2) = w;
        
        // 计算世界坐标系中的基向量
        Eigen::Vector3d U = (p2_w - p1_w).normalized();
        Eigen::Vector3d V_temp = (p3_w - p1_w);
        Eigen::Vector3d V = (V_temp - V_temp.dot(U) * U).normalized();
        Eigen::Vector3d W = U.cross(V);
        
        Eigen::Matrix3d R_world;
        R_world.col(0) = U;
        R_world.col(1) = V;
        R_world.col(2) = W;
        
        Eigen::Matrix3d R = R_world * R_local.transpose();
        Eigen::Vector3d t = p1_w - R * q1_l;
        
        verifyResult(world_points, local_points, R, t);
        
        Pose6D pose;
        pose.rotation = R;
        pose.translation = t;
        computeEulerAngles(R, pose.roll, pose.pitch, pose.yaw);
        
        return pose;
    }
    
    static void computeEulerAngles(const Eigen::Matrix3d& R, double& roll, double& pitch, double& yaw) {
        const double epsilon = 1e-6;
        pitch = atan2(-R(2,0), sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0)));
        
        if (fabs(pitch - M_PI/2) < epsilon) {
            yaw = 0;
            roll = atan2(R(0,1), R(1,1));
        } else if (fabs(pitch + M_PI/2) < epsilon) {
            yaw = 0;
            roll = atan2(-R(0,1), -R(1,1));
        } else {
            yaw = atan2(R(1,0)/cos(pitch), R(0,0)/cos(pitch));
            roll = atan2(R(2,1)/cos(pitch), R(2,2)/cos(pitch));
        }
    }
    
    static void verifyResult(const std::vector<Point3D>& world_points,
                            const std::vector<Point3D>& local_points,
                            const Eigen::Matrix3d& R,
                            const Eigen::Vector3d& t) {
        std::cout << "\n【验证结果 - 重投影误差】" << std::endl;
        double max_error = 0;
        
        for (size_t i = 0; i < world_points.size(); i++) {
            Eigen::Vector3d q_l(local_points[i].x, local_points[i].y, local_points[i].z);
            Eigen::Vector3d projected = R * q_l + t;
            
            double error = sqrt(pow(projected.x() - world_points[i].x, 2) +
                              pow(projected.y() - world_points[i].y, 2) +
                              pow(projected.z() - world_points[i].z, 2));
            
            std::cout << "  点 " << i+1 << ": 误差 = " << error*1000 << " mm" << std::endl;
            max_error = std::max(max_error, error);
        }
        std::cout << "  最大误差: " << max_error*1000 << " mm" << std::endl;
    }
};

// ========== YAML解析器（简化版）==========
class SimpleYAMLParser {
public:
    static bool parseYAML(const std::string& filename, 
                          std::vector<PoseEstimator::Point3D>& world_points,
                          std::vector<PoseEstimator::Point3D>& local_points) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "错误: 无法打开文件 " << filename << std::endl;
            return false;
        }
        
        std::string line;
        std::string current_section;
        std::vector<PoseEstimator::Point3D>* current_points = nullptr;
        
        while (std::getline(file, line)) {
            // 去除首尾空格
            size_t start = line.find_first_not_of(" \t\r\n");
            if (start == std::string::npos) continue; // 空行跳过
            if (line[start] == '#') continue; // 注释跳过
            
            std::string trimmed = line.substr(start);
            
            // 检查是否是节标题
            if (trimmed.find("world_points:") == 0) {
                current_section = "world";
                current_points = &world_points;
                continue;
            } else if (trimmed.find("local_points:") == 0) {
                current_section = "local";
                current_points = &local_points;
                continue;
            }
            
            // 解析点数据
            if (current_points != nullptr && (trimmed.find("- [") == 0 || trimmed.find("- [") == 0)) {
                // 格式: - [x, y, z]
                size_t bracket_start = trimmed.find('[');
                size_t bracket_end = trimmed.find(']');
                if (bracket_start != std::string::npos && bracket_end != std::string::npos) {
                    std::string coords = trimmed.substr(bracket_start + 1, bracket_end - bracket_start - 1);
                    // 替换逗号为空格
                    for (char& c : coords) {
                        if (c == ',') c = ' ';
                    }
                    std::stringstream ss(coords);
                    double x, y, z;
                    if (ss >> x >> y >> z) {
                        current_points->push_back(PoseEstimator::Point3D(x, y, z));
                    }
                }
            }
            // 也支持格式: - x, y, z 或 - x y z
            else if (current_points != nullptr && trimmed.find("- ") == 0) {
                std::string coords = trimmed.substr(2);
                for (char& c : coords) {
                    if (c == ',') c = ' ';
                }
                std::stringstream ss(coords);
                double x, y, z;
                if (ss >> x >> y >> z) {
                    current_points->push_back(PoseEstimator::Point3D(x, y, z));
                }
            }
        }
        
        file.close();
        return true;
    }
};

// ========== 生成JSON格式外参（用于fusion节点）==========
void printJSONExtrinsics(const PoseEstimator::Pose6D& pose) {
    std::cout << "\n【JSON格式外参 - 用于 fusion 节点】" << std::endl;
    std::cout << "{" << std::endl;
    std::cout << "  \"rotation\": [" << std::endl;
    std::cout << "    " << std::fixed << std::setprecision(9) 
              << pose.rotation(0,0) << ", " << pose.rotation(0,1) << ", " << pose.rotation(0,2) << "," << std::endl;
    std::cout << "    " << pose.rotation(1,0) << ", " << pose.rotation(1,1) << ", " << pose.rotation(1,2) << "," << std::endl;
    std::cout << "    " << pose.rotation(2,0) << ", " << pose.rotation(2,1) << ", " << pose.rotation(2,2) << std::endl;
    std::cout << "  ]," << std::endl;
    std::cout << "  \"translation\": [" << pose.translation.x() << ", " 
              << pose.translation.y() << ", " << pose.translation.z() << "]" << std::endl;
    std::cout << "}" << std::endl;
}

// ========== 主函数 ==========
int main(int argc, char* argv[]) {
    std::cout << "========================================" << std::endl;
    std::cout << "    全站仪位姿解算    " << std::endl;
    std::cout << "========================================" << std::endl;
    
    std::string config_file;
    
    // 解析命令行参数
    if (argc < 2) {
        std::cout << "\n使用方法: " << argv[0] << " <config.yaml>" << std::endl;
        std::cout << "\n示例配置文件格式 (config.yaml):" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "# 全站仪测量的世界坐标（单位：米）" << std::endl;
        std::cout << "world_points:" << std::endl;
        std::cout << "  - [1.234, 2.345, 0.567]  # 反光贴1" << std::endl;
        std::cout << "  - [1.567, 2.456, 0.589]  # 反光贴2" << std::endl;
        std::cout << "  - [1.345, 2.678, 0.598]  # 反光贴3" << std::endl;
        std::cout << std::endl;
        std::cout << "# 传感器局部坐标（单位：米）" << std::endl;
        std::cout << "local_points:" << std::endl;
        std::cout << "  - [0.000, 0.000, 0.000]  # 对应反光贴1" << std::endl;
        std::cout << "  - [0.100, 0.000, 0.000]  # 对应反光贴2" << std::endl;
        std::cout << "  - [0.000, 0.080, 0.000]  # 对应反光贴3" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "\n使用默认示例数据运行..." << std::endl;
        
        // 使用默认示例数据
        std::vector<PoseEstimator::Point3D> world_points = {
            PoseEstimator::Point3D(1.234, 2.345, 0.567),
            PoseEstimator::Point3D(1.567, 2.456, 0.589),
            PoseEstimator::Point3D(1.345, 2.678, 0.598)
        };
        
        std::vector<PoseEstimator::Point3D> local_points = {
            PoseEstimator::Point3D(0.000, 0.000, 0.000),
            PoseEstimator::Point3D(0.100, 0.000, 0.000),
            PoseEstimator::Point3D(0.000, 0.080, 0.000)
        };
        
        std::cout << "\n【输入数据 - 世界坐标】" << std::endl;
        for (size_t i = 0; i < world_points.size(); i++) {
            std::cout << "  点" << i+1 << ": (" << world_points[i].x << ", " 
                      << world_points[i].y << ", " << world_points[i].z << ")" << std::endl;
        }
        
        std::cout << "\n【输入数据 - 局部坐标】" << std::endl;
        for (size_t i = 0; i < local_points.size(); i++) {
            std::cout << "  点" << i+1 << ": (" << local_points[i].x << ", " 
                      << local_points[i].y << ", " << local_points[i].z << ")" << std::endl;
        }
        
        PoseEstimator::Pose6D pose = PoseEstimator::computePose(world_points, local_points);
        pose.print();
        printJSONExtrinsics(pose);
        
        return 0;
    }
    
    config_file = argv[1];
    std::cout << "\n读取配置文件: " << config_file << std::endl;
    
    std::vector<PoseEstimator::Point3D> world_points, local_points;
    
    if (!SimpleYAMLParser::parseYAML(config_file, world_points, local_points)) {
        std::cerr << "解析配置文件失败" << std::endl;
        return -1;
    }
    
    // 检查点数
    if (world_points.size() < 3) {
        std::cerr << "错误: world_points 至少需要3个点，当前只有 " << world_points.size() << " 个点" << std::endl;
        return -1;
    }
    
    if (local_points.size() < 3) {
        std::cerr << "错误: local_points 至少需要3个点，当前只有 " << local_points.size() << " 个点" << std::endl;
        return -1;
    }
    
    // 显示读取的数据
    std::cout << "\n【输入数据 - 世界坐标（全站仪测量）】" << std::endl;
    for (size_t i = 0; i < world_points.size(); i++) {
        std::cout << "  点" << i+1 << ": (" << std::fixed << std::setprecision(6)
                  << world_points[i].x << ", " << world_points[i].y << ", " << world_points[i].z << ")" << std::endl;
    }
    
    std::cout << "\n【输入数据 - 局部坐标（CAD模型/实测）】" << std::endl;
    for (size_t i = 0; i < local_points.size(); i++) {
        std::cout << "  点" << i+1 << ": (" << local_points[i].x << ", " 
                  << local_points[i].y << ", " << local_points[i].z << ")" << std::endl;
    }
    
    // 如果点数超过3个，只使用前3个
    if (world_points.size() > 3) {
        std::cout << "\n警告: 只使用前3个世界坐标点" << std::endl;
        world_points.resize(3);
    }
    
    if (local_points.size() > 3) {
        std::cout << "警告: 只使用前3个局部坐标点" << std::endl;
        local_points.resize(3);
    }
    
    // 计算位姿
    PoseEstimator::Pose6D pose = PoseEstimator::computePose(world_points, local_points);
    
    // 输出结果
    pose.print();
    
    // 输出JSON格式外参
    //printJSONExtrinsics(pose);
    
    return 0;
}