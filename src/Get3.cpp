#include <iostream>
#include <cmath>
#include <iomanip>
#include <string>
#include <sstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

/**
 * 全站仪坐标计算类 - 使用Eigen库
 * 坐标系定义：
 * - 原点：测站点在地面的投影点
 * - X轴：后视方向
 * - Y轴：垂直于后视方向（右手坐标系）
 * - Z轴：铅垂线向上为正
 */
class TotalStationCalculator {
private:
    double instrument_height;  // 仪器高 (m)
    double slope_distance;      // 斜距 (m)
    double horizontal_angle;    // 水平角 (度)
    double vertical_angle;      // 垂直角 (度)
    
public:
    // 角度转弧度
    double degToRad(double degrees) const {
        return degrees * M_PI / 180.0;
    }
    
    // 度分秒转十进制度数
    static double dmsToDecimal(int degrees, int minutes, double seconds) {
        double decimal = std::abs(degrees) + minutes / 60.0 + seconds / 3600.0;
        return degrees < 0 ? -decimal : decimal;
    }
    
    // 十进制度数转度分秒
    static void decimalToDms(double decimal, int& degrees, int& minutes, double& seconds) {
        degrees = static_cast<int>(decimal);
        double remainder = std::abs(decimal - degrees);
        minutes = static_cast<int>(remainder * 60);
        seconds = (remainder * 60 - minutes) * 60;
    }
    
    // 解析角度字符串（支持度、度分、度分秒格式）
    static double parseAngleString(const std::string& angleStr) {
        double degrees = 0, minutes = 0, seconds = 0;
        int sign = 1;
        size_t start = 0;
        
        // 处理负号
        if (angleStr[0] == '-') {
            sign = -1;
            start = 1;
        }
        
        // 查找度、分、秒分隔符
        size_t degPos = angleStr.find_first_of("°dD");
        size_t minPos = angleStr.find_first_of("'mM");
        size_t secPos = angleStr.find_first_of("\"sS");
        
        if (degPos != std::string::npos) {
            // 度格式
            degrees = std::stod(angleStr.substr(start, degPos - start));
            
            if (minPos != std::string::npos) {
                // 有分
                minutes = std::stod(angleStr.substr(degPos + 1, minPos - degPos - 1));
                
                if (secPos != std::string::npos) {
                    // 有秒
                    seconds = std::stod(angleStr.substr(minPos + 1, secPos - minPos - 1));
                }
            }
        } else {
            // 纯十进制格式
            degrees = std::stod(angleStr.substr(start));
        }
        
        return sign * (degrees + minutes / 60.0 + seconds / 3600.0);
    }
    
    // 构造函数
    TotalStationCalculator() 
        : instrument_height(0.0), slope_distance(0.0), 
          horizontal_angle(0.0), vertical_angle(0.0) {}
    
    TotalStationCalculator(double h, double s, double ha, double va)
        : instrument_height(h), slope_distance(s), 
          horizontal_angle(ha), vertical_angle(va) {}
    
    // 设置参数
    void setInstrumentHeight(double height) { instrument_height = height; }
    void setSlopeDistance(double distance) { slope_distance = distance; }
    void setHorizontalAngle(double angle) { horizontal_angle = angle; }
    void setVerticalAngle(double angle) { vertical_angle = angle; }
    
    // 获取参数
    double getInstrumentHeight() const { return instrument_height; }
    double getSlopeDistance() const { return slope_distance; }
    double getHorizontalAngle() const { return horizontal_angle; }
    double getVerticalAngle() const { return vertical_angle; }
    
    /**
     * 计算目标点的三维坐标（使用Eigen向量）
     * @return Eigen::Vector3d 包含 (X, Y, Z) 的向量
     */
    Eigen::Vector3d calculatePoint() const {
        double vert_rad = degToRad(vertical_angle);
        double horiz_rad = degToRad(horizontal_angle);
        
        // 计算平距和高差
        double horizontal_distance = slope_distance * std::cos(vert_rad);
        double vertical_difference = slope_distance * std::sin(vert_rad);
        
        // 计算坐标
        double x = horizontal_distance * std::cos(horiz_rad);
        double y = horizontal_distance * std::sin(horiz_rad);
        double z = instrument_height + vertical_difference;
        
        return Eigen::Vector3d(x, y, z);
    }
    
    /**
     * 使用旋转矩阵方法计算坐标（更符合测绘原理）
     * @return Eigen::Vector3d 目标点坐标
     */
    Eigen::Vector3d calculatePointWithRotation() const {
        double vert_rad = degToRad(vertical_angle);
        double horiz_rad = degToRad(horizontal_angle);
        
        Eigen::Vector3d local_point;
        local_point.x() = slope_distance * std::cos(vert_rad) * std::cos(horiz_rad);
        local_point.y() = slope_distance * std::cos(vert_rad) * std::sin(horiz_rad);
        local_point.z() = slope_distance * std::sin(vert_rad);
        
        Eigen::Vector3d ground_point;
        ground_point.x() = local_point.x();
        ground_point.y() = local_point.y();
        ground_point.z() = instrument_height + local_point.z();
        
        return ground_point;
    }
    
    /**
     * 批量计算多个目标点
     */
    static Eigen::MatrixXd batchCalculate(const Eigen::MatrixXd& params) {
        int n = params.rows();
        Eigen::MatrixXd results(n, 3);
        
        for (int i = 0; i < n; ++i) {
            TotalStationCalculator calc(
                params(i, 0), params(i, 1), params(i, 2), params(i, 3)
            );
            Eigen::Vector3d point = calc.calculatePoint();
            results.row(i) = point.transpose();
        }
        
        return results;
    }
    
    /**
     * 计算两点之间的距离
     */
    static double calculateDistance(const Eigen::Vector3d& point1, 
                                    const Eigen::Vector3d& point2) {
        return (point1 - point2).norm();
    }
    
    /**
     * 计算点的方位角（从X轴正方向起算）
     */
    static double calculateAzimuth(const Eigen::Vector3d& point) {
        return std::atan2(point.y(), point.x()) * 180.0 / M_PI;
    }
    
    /**
     * 计算点的天顶距（从Z轴正方向起算）
     */
    static double calculateZenithAngle(const Eigen::Vector3d& point) {
        double horizontal_distance = std::sqrt(point.x() * point.x() + point.y() * point.y());
        return std::atan2(horizontal_distance, point.z()) * 180.0 / M_PI;
    }
    
    // 显示角度（度分秒格式）
    void displayAngleDms(double angle, const std::string& name) const {
        int degrees, minutes;
        double seconds;
        decimalToDms(angle, degrees, minutes, seconds);
        std::cout << name << ": " << degrees << "°" << minutes << "'" 
                  << std::fixed << std::setprecision(3) << seconds << "\"" << std::endl;
    }
    
    // 显示计算结果
    void displayResults() const {
        Eigen::Vector3d point = calculatePoint();
        
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "========== 计算结果 ==========" << std::endl;
        std::cout << "X 坐标: " << std::setw(12) << point.x() << " m" << std::endl;
        std::cout << "Y 坐标: " << std::setw(12) << point.y() << " m" << std::endl;
        std::cout << "Z 坐标: " << std::setw(12) << point.z() << " m" << std::endl;
        
        // 额外输出平距和高差
        double vert_rad = degToRad(vertical_angle);
        double horizontal_distance = slope_distance * std::cos(vert_rad);
        double vertical_difference = slope_distance * std::sin(vert_rad);
        std::cout << "平距:   " << std::setw(12) << horizontal_distance << " m" << std::endl;
        std::cout << "高差:   " << std::setw(12) << vertical_difference << " m" << std::endl;
        std::cout << "===============================" << std::endl;
    }
    
    void displayInputs() const {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "========== 输入参数 ==========" << std::endl;
        std::cout << "仪器高 (i):      " << instrument_height << " m" << std::endl;
        std::cout << "斜距 (S):        " << slope_distance << " m" << std::endl;
        
        // 角度同时显示十进制和度分秒
        int deg, min;
        double sec;
        
        decimalToDms(horizontal_angle, deg, min, sec);
        std::cout << "水平角 (α):      " << horizontal_angle << "° = " 
                  << deg << "°" << min << "'" << std::fixed << std::setprecision(3) << sec << "\"" << std::endl;
        
        decimalToDms(vertical_angle, deg, min, sec);
        std::cout << "垂直角 (β):      " << vertical_angle << "° = " 
                  << deg << "°" << min << "'" << std::fixed << std::setprecision(3) << sec << "\"" << std::endl;
        
        std::cout << "===============================" << std::endl;
    }
};

// 交互式输入函数（支持度分秒）
void interactiveInput() {
    TotalStationCalculator calculator;
    std::string inputStr;
    double input;
    
    std::cout << "\n=== 全站仪坐标计算程序 ===" << std::endl;
    std::cout << "角度支持格式：\n";
    std::cout << "  - 十进制: 30.5\n";
    std::cout << "  - 度分秒: 30°30'30\" 或 30d30m30s\n";
    std::cout << std::endl;
    
    // 输入仪器高
    std::cout << "仪器高 (m): ";
    std::cin >> input;
    calculator.setInstrumentHeight(input);
    
    // 输入斜距
    std::cout << "斜距 (m): ";
    std::cin >> input;
    calculator.setSlopeDistance(input);
    
    // 输入水平角（支持度分秒）
    std::cout << "水平角 (度 或 度分秒): ";
    std::cin >> inputStr;
    calculator.setHorizontalAngle(TotalStationCalculator::parseAngleString(inputStr));
    
    // 输入垂直角（支持度分秒）
    std::cout << "垂直角 (度 或 度分秒): ";
    std::cin >> inputStr;
    calculator.setVerticalAngle(TotalStationCalculator::parseAngleString(inputStr));
    
    std::cout << std::endl;
    calculator.displayInputs();
    calculator.displayResults();
}

// 命令行参数模式（支持度分秒）
void commandLineMode(int argc, char* argv[]) {
    if (argc != 5) {
        std::cout << "用法: " << argv[0] << " <仪器高> <斜距> <水平角> <垂直角>" << std::endl;
        std::cout << "示例: " << std::endl;
        std::cout << "  " << argv[0] << " 1.5 50 45 30" << std::endl;
        std::cout << "  " << argv[0] << " 1.5 50 45d30m20s 30d15m10s" << std::endl;
        return;
    }
    
    try {
        double instrument_height = std::stod(argv[1]);
        double slope_distance = std::stod(argv[2]);
        
        // 解析角度（支持度分秒格式）
        double horizontal_angle = TotalStationCalculator::parseAngleString(argv[3]);
        double vertical_angle = TotalStationCalculator::parseAngleString(argv[4]);
        
        TotalStationCalculator calc(instrument_height, slope_distance, horizontal_angle, vertical_angle);
        calc.displayInputs();
        calc.displayResults();
    } catch (const std::exception& e) {
        std::cout << "输入参数错误: " << e.what() << std::endl;
        std::cout << "请检查输入格式！" << std::endl;
    }
}

// 批量计算示例
void batchCalculationExample() {
    std::cout << "\n=== 批量计算示例 ===" << std::endl;
    
    Eigen::MatrixXd params(5, 4);
    params << 1.500, 50.000, 45.0, 30.0,
              1.500, 30.000, 90.0, 15.0,
              1.500, 40.000, 180.0, 45.0,
              1.600, 25.000, 270.0, 10.0,
              1.550, 35.500, 120.0, 25.5;
    
    Eigen::MatrixXd results = TotalStationCalculator::batchCalculate(params);
    
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "点号\tX (m)\t\tY (m)\t\tZ (m)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    for (int i = 0; i < results.rows(); ++i) {
        std::cout << i+1 << "\t" 
                  << std::setw(10) << results(i, 0) << "\t"
                  << std::setw(10) << results(i, 1) << "\t"
                  << std::setw(10) << results(i, 2) << std::endl;
    }
}

// 主函数
int main(int argc, char* argv[]) {
    std::cout << "全站仪坐标计算" << std::endl;
    std::cout << "=====================================" << std::endl;
    
    if (argc == 5) {
        // 命令行参数模式
        commandLineMode(argc, argv);
    } else if (argc > 1) {
        // 参数数量错误
        std::cout << "错误：需要4个参数，实际提供了" << (argc-1) << "个" << std::endl;
        std::cout << "用法: " << argv[0] << " <仪器高> <斜距> <水平角> <垂直角>" << std::endl;
    } else {
        // 交互式输入模式
        interactiveInput();
        
        // 显示批量计算示例
        /*char choice;
        std::cout << "\n是否显示批量计算示例？(y/n): ";
        std::cin >> choice;
        if (choice == 'y' || choice == 'Y') {
            batchCalculationExample();
        }*/
    }
    
    return 0;
}