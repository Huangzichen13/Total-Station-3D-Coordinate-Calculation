#include <iostream>
#include <vector>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iomanip>

// ========== 函数声明（放在文件开头，main函数之前）==========
void customDrawFrameAxes(cv::Mat& image, 
                         const cv::Mat& camera_matrix,
                         const cv::Mat& dist_coeffs,
                         const cv::Mat& rvec,
                         const cv::Mat& tvec,
                         double length);

// ========== 自定义绘制坐标系轴函数 ==========
void customDrawFrameAxes(cv::Mat& image, 
                         const cv::Mat& camera_matrix,
                         const cv::Mat& dist_coeffs,
                         const cv::Mat& rvec,
                         const cv::Mat& tvec,
                         double length) {
    if (image.empty() || camera_matrix.empty() || rvec.empty() || tvec.empty()) {
        return;
    }
    
    // 定义三个轴的方向（在标定板坐标系中）
    std::vector<cv::Point3f> axes_points;
    axes_points.push_back(cv::Point3f(0, 0, 0));      // 原点
    axes_points.push_back(cv::Point3f(length, 0, 0)); // X轴
    axes_points.push_back(cv::Point3f(0, length, 0)); // Y轴
    axes_points.push_back(cv::Point3f(0, 0, length)); // Z轴
    
    // 投影到图像平面
    std::vector<cv::Point2f> image_points;
    try {
        cv::projectPoints(axes_points, rvec, tvec, camera_matrix, dist_coeffs, image_points);
    } catch (const cv::Exception& e) {
        std::cerr << "projectPoints failed: " << e.what() << std::endl;
        return;
    }
    
    if (image_points.size() < 4) {
        return;
    }
    
    // 绘制轴线（BGR颜色）
    cv::Scalar x_color(0, 0, 255);   // X轴: 红色
    cv::Scalar y_color(0, 255, 0);   // Y轴: 绿色
    cv::Scalar z_color(255, 0, 0);   // Z轴: 蓝色
    
    cv::line(image, image_points[0], image_points[1], x_color, 3);
    cv::line(image, image_points[0], image_points[2], y_color, 3);
    cv::line(image, image_points[0], image_points[3], z_color, 3);
    
    // 添加文本标签
    cv::putText(image, "X", image_points[1], cv::FONT_HERSHEY_SIMPLEX, 0.6, x_color, 2);
    cv::putText(image, "Y", image_points[2], cv::FONT_HERSHEY_SIMPLEX, 0.6, y_color, 2);
    cv::putText(image, "Z", image_points[3], cv::FONT_HERSHEY_SIMPLEX, 0.6, z_color, 2);
}

// ========== BoardPoseEstimator 类 ==========
class BoardPoseEstimator {
public:
    struct PoseResult {
        bool success;
        cv::Mat rvec;
        cv::Mat tvec;
        cv::Mat rotation_matrix;
        cv::Mat transform_matrix;
        cv::Mat camera_position;
        double reprojection_error;
    };
    
    BoardPoseEstimator(cv::Size pattern_size, double square_size)
        : pattern_size_(pattern_size), square_size_(square_size) {
        buildBoardPoints();
    }
    
    PoseResult estimatePose(const cv::Mat& image,
                            const cv::Mat& camera_matrix,
                            const cv::Mat& dist_coeffs) {
        PoseResult result;
        result.success = false;
        result.reprojection_error = 0.0;
        
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, pattern_size_, corners,
                                                cv::CALIB_CB_ADAPTIVE_THRESH |
                                                cv::CALIB_CB_NORMALIZE_IMAGE);
        
        if (!found) {
            std::cerr << "未检测到棋盘格角点" << std::endl;
            return result;
        }
        
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
        
        bool success = cv::solvePnP(board_points_3d_, corners,
                                    camera_matrix, dist_coeffs,
                                    result.rvec, result.tvec, false,
                                    cv::SOLVEPNP_ITERATIVE);
        
        if (!success) {
            std::cerr << "PnP求解失败" << std::endl;
            return result;
        }
        
        result.reprojection_error = computeReprojectionError(
            board_points_3d_, corners, result.rvec, result.tvec,
            camera_matrix, dist_coeffs);
        
        cv::Rodrigues(result.rvec, result.rotation_matrix);
        
        result.transform_matrix = cv::Mat::eye(4, 4, CV_64F);
        result.rotation_matrix.copyTo(result.transform_matrix(cv::Rect(0, 0, 3, 3)));
        result.tvec.copyTo(result.transform_matrix(cv::Rect(3, 0, 1, 3)));
        
        result.camera_position = -result.rotation_matrix.t() * result.tvec;
        
        result.success = true;
        return result;
    }
    
    std::vector<cv::Point3f> getBoardPoints() const {
        return board_points_3d_;
    }
    
private:
    cv::Size pattern_size_;
    double square_size_;
    std::vector<cv::Point3f> board_points_3d_;
    
    void buildBoardPoints() {
        board_points_3d_.clear();
        for (int i = 0; i < pattern_size_.height; i++) {
            for (int j = 0; j < pattern_size_.width; j++) {
                board_points_3d_.emplace_back(
                    j * square_size_,
                    i * square_size_,
                    0.0
                );
            }
        }
    }
    
    double computeReprojectionError(const std::vector<cv::Point3f>& object_points,
                                    const std::vector<cv::Point2f>& image_points,
                                    const cv::Mat& rvec,
                                    const cv::Mat& tvec,
                                    const cv::Mat& camera_matrix,
                                    const cv::Mat& dist_coeffs) {
        std::vector<cv::Point2f> projected_points;
        cv::projectPoints(object_points, rvec, tvec,
                         camera_matrix, dist_coeffs, projected_points);
        
        double total_error = 0.0;
        for (size_t i = 0; i < image_points.size(); i++) {
            double error = cv::norm(image_points[i] - projected_points[i]);
            total_error += error;
        }
        
        return total_error / image_points.size();
    }
};

// ========== 辅助函数 ==========
void printPoseResult(const BoardPoseEstimator::PoseResult& result) {
    if (!result.success) {
        std::cout << "位姿估计失败！" << std::endl;
        return;
    }
    
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n========================================" << std::endl;
    std::cout << "    相机相对于标定板的位姿估计结果" << std::endl;
    std::cout << "========================================" << std::endl;
    
    std::cout << "\n【平移向量 tvec (米)】" << std::endl;
    std::cout << "  [" << result.tvec.at<double>(0) << ", "
              << result.tvec.at<double>(1) << ", "
              << result.tvec.at<double>(2) << "]" << std::endl;
    
    std::cout << "\n【相机位置 (在标定板坐标系中, 米)】" << std::endl;
    std::cout << "  X: " << result.camera_position.at<double>(0) << std::endl;
    std::cout << "  Y: " << result.camera_position.at<double>(1) << std::endl;
    std::cout << "  Z: " << result.camera_position.at<double>(2) << std::endl;
    
    std::cout << "\n【旋转矩阵 R (3x3)】" << std::endl;
    for (int i = 0; i < 3; i++) {
        std::cout << "  ";
        for (int j = 0; j < 3; j++) {
            std::cout << std::setw(12) << result.rotation_matrix.at<double>(i, j) << " ";
        }
        std::cout << std::endl;
    }
    
    std::cout << "\n【齐次变换矩阵 T_board_to_camera (4x4)】" << std::endl;
    for (int i = 0; i < 4; i++) {
        std::cout << "  ";
        for (int j = 0; j < 4; j++) {
            std::cout << std::setw(12) << result.transform_matrix.at<double>(i, j) << " ";
        }
        std::cout << std::endl;
    }
    
    std::cout << "\n【重投影误差】" << std::endl;
    std::cout << "  " << result.reprojection_error << " 像素" << std::endl;
    std::cout << "========================================" << std::endl;
}

void generateTestCameraMatrix(cv::Mat& camera_matrix, cv::Mat& dist_coeffs) {
    double fx = 1000.0;
    double fy = 1000.0;
    double cx = 640.0;
    double cy = 360.0;
    
    camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    dist_coeffs = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
}

int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "    PnP 相机位姿估计程序" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // 棋盘格参数
    cv::Size pattern_size(18, 12);
    double square_size = 0.045;  // 3cm
    
    // 相机内参
    cv::Mat camera_matrix, dist_coeffs;
    
    // 检查命令行参数
    if (argc < 2) {
        std::cout << "\n使用方法:" << std::endl;
        std::cout << "  1. 使用默认内参: " << argv[0] << " <图像文件>" << std::endl;
        std::cout << "  2. 使用标定文件: " << argv[0] << " <标定文件.yaml> <图像文件>" << std::endl;
        std::cout << "\n示例:" << std::endl;
        std::cout << "  " << argv[0] << " chessboard.png" << std::endl;
        std::cout << "  " << argv[0] << " camera.yaml chessboard.png" << std::endl;
        return -1;
    }
    
    std::string image_path;
    
    if (argc == 2) {
        // 使用默认内参
        std::cout << "\n使用默认相机内参（精度较低）" << std::endl;
        generateTestCameraMatrix(camera_matrix, dist_coeffs);
        image_path = argv[1];
    } else {
        // 从YAML文件读取内参
        std::string calib_file = argv[1];
        image_path = argv[2];
        
        std::cout << "\n加载相机标定文件: " << calib_file << std::endl;
        
        cv::FileStorage fs(calib_file, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cerr << "警告: 无法打开标定文件: " << calib_file << std::endl;
            std::cerr << "将使用默认内参" << std::endl;
            generateTestCameraMatrix(camera_matrix, dist_coeffs);
        } else {
            fs["camera_matrix"] >> camera_matrix;
            fs["distortion_coefficients"] >> dist_coeffs;
            fs.release();
            
            if (camera_matrix.empty()) {
                std::cerr << "警告: 标定文件中未找到 camera_matrix，使用默认内参" << std::endl;
                generateTestCameraMatrix(camera_matrix, dist_coeffs);
            } else {
                std::cout << "成功加载相机内参" << std::endl;
            }
        }
    }
    
    std::cout << "\n相机内参矩阵:" << std::endl;
    std::cout << camera_matrix << std::endl;
    
    // 创建估计器
    BoardPoseEstimator estimator(pattern_size, square_size);
    
    // 读取图像
    std::cout << "\n读取图像: " << image_path << std::endl;
    cv::Mat image = cv::imread(image_path);
    
    if (image.empty()) {
        std::cerr << "错误: 无法读取图像: " << image_path << std::endl;
        return -1;
    }
    
    // 估计位姿
    auto result = estimator.estimatePose(image, camera_matrix, dist_coeffs);
    
    if (result.success) {
        printPoseResult(result);
        
        // 可视化
        cv::Mat image_with_axis = image.clone();
        
        // 绘制棋盘格角点
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        cv::findChessboardCorners(gray, pattern_size, corners,
                                  cv::CALIB_CB_ADAPTIVE_THRESH |
                                  cv::CALIB_CB_NORMALIZE_IMAGE);
        cv::drawChessboardCorners(image_with_axis, pattern_size, corners, true);
        
        // 绘制坐标系轴
        customDrawFrameAxes(image_with_axis, camera_matrix, dist_coeffs,
                           result.rvec, result.tvec, square_size * 5);
        
        // 保存结果图像
        cv::imwrite("pose_result.png", image_with_axis);
        std::cout << "\n结果图像已保存到: pose_result.png" << std::endl;
        
        //cv::imshow("Camera Pose", image_with_axis);
        std::cout << "\n按任意键退出..." << std::endl;
        cv::waitKey(0);
    } else {
        std::cerr << "位姿估计失败！请确保:" << std::endl;
        std::cerr << "  - 棋盘格完整可见" << std::endl;
        std::cerr << "  - 棋盘格角点清晰" << std::endl;
        std::cerr << "  - 棋盘格参数正确 (内角点: " << pattern_size.width << "x" << pattern_size.height << ")" << std::endl;
        std::cerr << "  - 方格边长: " << square_size << "米" << std::endl;
    }
    
    return 0;
}