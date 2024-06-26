//
// Created by mulong on 2024/4/22.
//

#include <chrono>
#include <depthai/build/version.hpp>
#include <depthai/depthai.hpp>
#include <deque>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>


// 定义帧率常量
/*
 * mono 400p : max 50fps
 * mono 720p/800p : max 40fps
 * color 720p/800p : max 30fps
 */
constexpr int FPS = 30;

// 是否为彩色相机
constexpr bool color = true;
// 相机分辨率
const std::string resolution = "400";

// 定义相机列表
std::vector<std::string> cam_list{
    "CAM_A",
    "CAM_B",
    "CAM_C",
    "CAM_D",
};

// 黑白相机分辨率选项
std::map<std::string, dai::MonoCameraProperties::SensorResolution> mono_res_opts = {
    {"400", dai::MonoCameraProperties::SensorResolution::THE_400_P},
    {"480", dai::MonoCameraProperties::SensorResolution::THE_480_P},
    {"720", dai::MonoCameraProperties::SensorResolution::THE_720_P},
    {"800", dai::MonoCameraProperties::SensorResolution::THE_800_P},
    {"1200", dai::MonoCameraProperties::SensorResolution::THE_1200_P},
};

// 彩色相机分辨率选项
std::map<std::string, dai::ColorCameraProperties::SensorResolution> color_res_opts = {
    {"720", dai::ColorCameraProperties::SensorResolution::THE_720_P},
    {"800", dai::ColorCameraProperties::SensorResolution::THE_800_P},
    {"1080", dai::ColorCameraProperties::SensorResolution::THE_1080_P},
    {"1200", dai::ColorCameraProperties::SensorResolution::THE_1200_P},
    {"4k", dai::ColorCameraProperties::SensorResolution::THE_4_K},
    {"5mp", dai::ColorCameraProperties::SensorResolution::THE_5_MP},
    {"12mp", dai::ColorCameraProperties::SensorResolution::THE_12_MP},
    {"48mp", dai::ColorCameraProperties::SensorResolution::THE_48_MP},
};

// 将相机板插槽名称映射到相机名称的字典
std::map<std::string, std::string> cam_socket_to_name = {
    {"RGB", "CAM_A"},
    {"LEFT", "CAM_B"},
    {"RIGHT", "CAM_C"},
    {"CAM_A", "CAM_A"},
    {"CAM_B", "CAM_B"},
    {"CAM_C", "CAM_C"},
    {"CAM_D", "CAM_D"},
    {"CAM_E", "CAM_E"},
    {"CAM_F", "CAM_F"},
};

// 相机板插槽选项
std::map<std::string, dai::CameraBoardSocket> cam_socket_opts = {
    {"CAM_A", dai::CameraBoardSocket::CAM_A},
    {"CAM_B", dai::CameraBoardSocket::CAM_B},
    {"CAM_C", dai::CameraBoardSocket::CAM_C},
    {"CAM_D", dai::CameraBoardSocket::CAM_D},
    {"CAM_E", dai::CameraBoardSocket::CAM_E},
    {"CAM_F", dai::CameraBoardSocket::CAM_F},
};

/**
 * 帧率处理类，用于记录和计算不同事件的帧率。
 */
class FPSHandler {
   private:
    // 开始记录时间的时间点
    std::chrono::steady_clock::time_point start;
    // 以事件名称为键，存储时间戳的双端队列
    std::map<std::string, std::deque<double>> ticks;
    // 最大时间戳数量，用于控制双端队列的长度
    int maxTicks{100};

   public:
    /**
     * 构造函数，初始化开始记录时间的时间点。
     */
    FPSHandler() : start(std::chrono::steady_clock::now()) {}

    /**
     * 记录指定事件的当前时间戳。
     * @param name 事件的名称。
     */
    void tick(const std::string& name) {
        // 如果该事件时间戳队列不存在，则创建新队列
        if(ticks.find(name) == ticks.end()) {
            ticks[name] = std::deque<double>();
        }
        // 记录当前时间戳（单位：秒）
        ticks[name].push_back(static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count())
                              / 1000.0);

        while(ticks[name].size() > maxTicks) {
            ticks[name].pop_front();
        }
    }

    /**
     * 计算并返回指定事件的帧率。
     * @param name 事件的名称。
     * @return 指定事件的帧率，如果无法计算，则返回 0.0。
     */
    double tick_fps(const std::string& name) {
        // 确保事件存在且时间戳数量大于 1
        if(ticks.find(name) != ticks.end() && ticks[name].size() > 1) {
            // 计算时间差
            auto const time_diff = ticks[name].back() - ticks[name].front();  // 计算时间差
            return static_cast<double>(ticks[name].size() - 1) / time_diff;   // 计算帧率
        }
        return 0.0;
    }

    /**
     * 打印所有事件的帧率状态。
     */
    void print_status() {
        std::cout << "=== TOTAL FPS ===" << '\n';
        for(const auto& pair : ticks) {
            std::cout << "[" << pair.first << "]: " << tick_fps(pair.first) << '\n';
        }
    }

    /**
     * 在给定的 OpenCV 帧上绘制指定事件的帧率。
     * @param frame 要绘制帧率的 OpenCV 帧。
     * @param name 事件的名称。
     */
    void draw_fps(cv::Mat& frame, const std::string& name) {
        // 计算并格式化帧率字符串
        // std::string fps_str = name + " FPS: " + std::to_string(tick_fps(name));
        std::string fps_str = cv::format("%s FPS: %.3f", name.c_str(),tick_fps(name) );
        fps_str.erase(fps_str.find_last_not_of('0') + 1, std::string::npos);
        fps_str.erase(fps_str.find_last_not_of('.') + 1, std::string::npos);
        // 在帧上绘制帧率
        cv::putText(frame, fps_str, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(128, 128, 128), 4, cv::LINE_AA);
        cv::putText(frame, fps_str, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

        if(name != "nn") {
            auto const nn_fps = tick_fps("nn");
            if(nn_fps > 0) {
                std::string nn_fps_str = "NN FPS: " + std::to_string(nn_fps);

                nn_fps_str.erase(nn_fps_str.find_last_not_of('0') + 1, std::string::npos);
                nn_fps_str.erase(nn_fps_str.find_last_not_of('.') + 1, std::string::npos);

                cv::putText(frame, nn_fps_str, cv::Point(10, 45), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(128, 128, 128), 4, cv::LINE_AA);
                cv::putText(frame, nn_fps_str, cv::Point(10, 45), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            }
        }
    }
};

// 格式化持续时间
std::string formatDuration(const std::chrono::steady_clock::duration duration) {
    // 首先，将 duration 转换为纳秒
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);

    // 计算小时、分钟、秒和毫秒
    auto const hrs = std::chrono::duration_cast<std::chrono::hours>(ns);
    ns -= hrs;
    auto const mins = std::chrono::duration_cast<std::chrono::minutes>(ns);
    ns -= mins;
    auto const secs = std::chrono::duration_cast<std::chrono::seconds>(ns);
    ns -= secs;
    auto const millis = std::chrono::duration_cast<std::chrono::milliseconds>(ns);
    ns -= millis;
    auto const micros = std::chrono::duration_cast<std::chrono::microseconds>(ns);
    ns -= micros;

    // 构建格式化的字符串
    std::ostringstream oss;
    oss << std::setw(2) << std::setfill('0') << hrs.count() << ":"   // 时
        << std::setw(2) << std::setfill('0') << mins.count() << ":"  // 分
        << std::setw(2) << std::setfill('0') << secs.count() << "."  // 秒
        << std::setw(3) << std::setfill('0') << millis.count()       // 毫秒
        << std::setw(3) << std::setfill('0') << micros.count();

    return oss.str();
}

int main() {
    dai::Device device;

    std::cout << "Depthai Core: " << dai::build::VERSION << '\n';

    std::cout << "Usb speed: " << device.getUsbSpeed() << '\n';

    // 创建深度 AI 管道
    dai::Pipeline pipeline;
    pipeline.setXLinkChunkSize(0);

    // 创建用于存储相机输出的 XLinkOut 节点的字典
    std::map<std::string, std::shared_ptr<dai::node::ColorCamera>> camColor;
    std::map<std::string, std::shared_ptr<dai::node::MonoCamera>> camMono;

    auto sync = pipeline.create<dai::node::Sync>();
    // sync->setSyncThreshold(std::chrono::seconds(1));
    auto xOut = pipeline.create<dai::node::XLinkOut>();
    xOut->setStreamName("msgOut");
    sync->out.link(xOut->input);
    // 遍历相机列表并配置管道
    for(const auto& cam_name : cam_list) {
        if(color) {
            camColor[cam_name] = pipeline.create<dai::node::ColorCamera>();
            camColor[cam_name]->setResolution(color_res_opts[resolution]);
            camColor[cam_name]->setBoardSocket(cam_socket_opts[cam_name]);
            camColor[cam_name]->setFps(FPS);
            camColor[cam_name]->isp.link(sync->inputs[cam_name]);
            camColor[cam_name]->initialControl.setFrameSyncMode(cam_name == "CAM_A" ? dai::CameraControl::FrameSyncMode::OUTPUT
                                                                                    : dai::CameraControl::FrameSyncMode::INPUT);
        } else {
            camMono[cam_name] = pipeline.create<dai::node::MonoCamera>();
            camMono[cam_name]->setResolution(mono_res_opts[resolution]);
            camMono[cam_name]->setBoardSocket(cam_socket_opts[cam_name]);
            camMono[cam_name]->setFps(FPS);
            camMono[cam_name]->out.link(sync->inputs[cam_name]);
            camMono[cam_name]->initialControl.setFrameSyncMode(cam_name == "CAM_A" ? dai::CameraControl::FrameSyncMode::OUTPUT
                                                                                   : dai::CameraControl::FrameSyncMode::INPUT);
        }
    }

    // 启动管道
    device.startPipeline(pipeline);

    auto const msgGrp = device.getOutputQueue("msgOut", 4, false);

    FPSHandler fps_handler;

    while(true) {
        std::cout << "--------------------" << '\n';
        auto msgData = msgGrp->get<dai::MessageGroup>();
        if(msgData == nullptr) {
            continue;
        }
        for(const auto& cam_name : cam_list) {
            // 从设备获取相机输出数据包
            auto packet = msgData->get<dai::ImgFrame>(cam_name);
            if(packet) {
                fps_handler.tick(cam_name);
                std::cout << "Received frame " << cam_name << ": " << formatDuration(packet->getTimestampDevice().time_since_epoch()) << '\n';

                auto frame = packet->getCvFrame();
                fps_handler.draw_fps(frame, cam_name);
                cv::imshow(cam_name, frame);
            }
        }
        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
