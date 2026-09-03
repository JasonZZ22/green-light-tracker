/**
 * RM 机甲大师 制导飞镖 —— 绿光视觉识别 + MAVLink 串口输出
 *
 * 功能：
 *   1. OV5647 摄像头 640x480 实时采集
 *   2. HSV 阈值法检测绿色引导光源 → 质心坐标
 *   3. 坐标 → 舵机 PWM → MAVLink RC_CHANNELS_OVERRIDE → 串口发给飞控
 */

#include "green_detector.h"
#include "serial_comm.h"

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <atomic>
#include <csignal>

static std::atomic<bool> running{true};
void sigHandler(int) { running = false; }

// ====================== 安全的 GUI 包装层 ======================
static bool gui_ok = false;

static void probeGUI() {
    if (!std::getenv("DISPLAY") || std::getenv("QT_QPA_PLATFORM")) {
        gui_ok = false;
        return;
    }
    try {
        cv::namedWindow("_probe", cv::WINDOW_GUI_EXPANDED);
        cv::destroyWindow("_probe");
        gui_ok = true;
    } catch (...) { gui_ok = false; }
}

static void safeImshow(const char* w, const cv::Mat& m) {
    if (!gui_ok) return;
    try { cv::imshow(w, m); } catch (...) { gui_ok = false; }
}
static int safeWaitKey(int ms) {
    if (!gui_ok) return -1;
    try { return cv::waitKey(ms) & 0xFF; } catch (...) { gui_ok = false; return -1; }
}
static void safeDestroyAll() {
    if (!gui_ok) return;
    try { cv::destroyAllWindows(); } catch (...) { gui_ok = false; }
}

// ====================== 轨道条 ======================
static int h_low = 35, h_high = 85;
static int s_low = 80, s_high = 255;
static int v_low = 80, v_high = 255;
static int morph_k = 3;
static int min_area = 50;

static void createTrackbars() {
    if (!gui_ok) return;
    try {
        cv::namedWindow("Green Light Tracker", cv::WINDOW_AUTOSIZE);
        cv::createTrackbar("H Low",  "Green Light Tracker", &h_low,  180);
        cv::createTrackbar("H High", "Green Light Tracker", &h_high, 180);
        cv::createTrackbar("S Low",  "Green Light Tracker", &s_low,  255);
        cv::createTrackbar("S High", "Green Light Tracker", &s_high, 255);
        cv::createTrackbar("V Low",  "Green Light Tracker", &v_low,  255);
        cv::createTrackbar("V High", "Green Light Tracker", &v_high, 255);
        cv::createTrackbar("Morph K", "Green Light Tracker", &morph_k, 9);
        cv::createTrackbar("Min Area", "Green Light Tracker", &min_area, 500);
    } catch (...) { gui_ok = false; }
}

// ====================== 模式 0: 摄像头 + 串口 ======================
static int runCamera(int cam_id, const char* serial_dev) {
    // ---- 摄像头 (GStreamer 管道, 兼容 rkisp) ----
    cv::VideoCapture cap;
    char pipeline[256];
    std::snprintf(pipeline, sizeof(pipeline),
        "v4l2src device=/dev/video%d io-mode=mmap ! "
        "video/x-raw,width=640,height=480 ! "
        "videoconvert ! video/x-raw,format=BGR ! appsink",
        cam_id);

    bool opened = cap.open(pipeline, cv::CAP_GSTREAMER);
    if (!opened) opened = cap.open(cam_id, cv::CAP_V4L2);
    if (!opened) {
        std::cerr << "无法打开摄像头 /dev/video" << cam_id << std::endl;
        return -1;
    }

    double fw = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double fh = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    if (fw < 1) fw = 640;
    if (fh < 1) fh = 480;
    const float frame_cx = fw / 2.0f;
    const float frame_cy = fh / 2.0f;
    std::cout << "摄像头: " << fw << "x" << fh << std::endl;

    // ---- 串口 (MAVLink → 飞控) ----
    SerialComm serial;
    bool serial_ok = false;
    if (serial_dev && serial_dev[0]) {
        serial_ok = serial.open(serial_dev, 115200);
        if (serial_ok)
            serial.sendServoCmd(1500, 1500, 1500, 1500);  // 归中
    }

    createTrackbars();
    GreenDetector detector;
    cv::Mat frame;
    long long idx = 0;

    while (running) {
        cap >> frame;
        if (frame.empty()) continue;
        ++idx;

        GreenDetector::HSVRange hsv;
        if (gui_ok) {
            hsv.h_low  = h_low;   hsv.h_high = h_high;
            hsv.s_low  = s_low;   hsv.s_high = s_high;
            hsv.v_low  = v_low;   hsv.v_high = v_high;
            detector.setMorphKernelSize(morph_k > 0 ? morph_k : 1);
            detector.setMinArea(static_cast<float>(min_area));
        }
        detector.setHSV(hsv);

        auto result = detector.detect(frame, true);

        // ---- 检测到目标 → MAVLink 发给飞控 ----
        if (result.found) {
            // 像素偏差映射到 PWM 偏转量（中心=1500）
            // 假设画面覆盖 ±30° 范围，映射到 PWM 1000-2000
            const float deg_per_px = 60.0f / fw;   // 每像素约 0.09°
            float err_x = result.cx - frame_cx;      // 正值=目标在右
            float err_y = result.cy - frame_cy;      // 正值=目标在下

            uint16_t ch1 = static_cast<uint16_t>(1500 + err_x * deg_per_px * (1000.0f / 90.0f));  // Roll / Pan
            uint16_t ch2 = static_cast<uint16_t>(1500 + err_y * deg_per_px * (1000.0f / 90.0f));  // Pitch / Tilt

            // 限幅 1000~2000
            if (ch1 < 1000) ch1 = 1000; if (ch1 > 2000) ch1 = 2000;
            if (ch2 < 1000) ch2 = 1000; if (ch2 > 2000) ch2 = 2000;

            serial.sendServoCmd(ch1, ch2, 1500, 1500);

            std::cout << "TARGET: (" << result.cx << ", " << result.cy
                      << ") r=" << result.radius
                      << " ch1=" << ch1 << " ch2=" << ch2
                      << "\r" << std::flush;
        } else {
            // 无目标时也发送归中指令，保持串口数据流连续
            serial.sendServoCmd(1500, 1500, 1500, 1500);
            std::cout << "NO TARGET              \r" << std::flush;
        }

        // ---- GUI / 截图 ----
        safeImshow("Green Light Tracker", frame);
        int key = safeWaitKey(1);
        if (key == 27) break;
        if (key == 's' || key == 'S') {
            cv::imwrite("capture.png", frame);
            cv::imwrite("mask.png", detector.debugMask());
            std::cout << "\n截图: capture.png / mask.png\n";
        }
        if (!gui_ok && idx % 60 == 0) {
            cv::imwrite("/tmp/green_tracker_frame.png", frame);
            cv::imwrite("/tmp/green_tracker_mask.png", detector.debugMask());
        }
    }

    cap.release();
    safeDestroyAll();
    return 0;
}

// ====================== 模式 1: 图片测试 ======================
static int runImage(const char* path) {
    cv::Mat frame = cv::imread(path);
    if (frame.empty()) {
        std::cerr << "无法读取图片: " << path << std::endl;
        return -1;
    }
    std::cout << "图片尺寸: " << frame.cols << "x" << frame.rows << std::endl;

    GreenDetector detector;
    auto result = detector.detect(frame, true);
    if (result.found)
        std::cout << "检测到绿光: (" << result.cx << ", " << result.cy
                  << ") 半径=" << result.radius << " 面积=" << result.area << std::endl;
    else
        std::cout << "未检测到绿光目标\n";

    cv::imwrite("result.png", frame);
    cv::imwrite("result_mask.png", detector.debugMask());
    std::cout << "结果已保存: result.png / result_mask.png\n";

    safeImshow("Result", frame);
    safeImshow("Mask", detector.debugMask());
    if (gui_ok) {
        std::cout << "按任意键退出...\n";
        for (int i = 0; i < 30; ++i) { if (safeWaitKey(100) >= 0) break; }
    }
    safeDestroyAll();
    return 0;
}

// ====================== 模式 2: 视频文件 ======================
static int runVideo(const char* path) {
    cv::VideoCapture cap(path);
    if (!cap.isOpened()) { std::cerr << "无法打开视频: " << path << std::endl; return -1; }
    createTrackbars();
    GreenDetector detector;
    cv::Mat frame;
    long long idx = 0;
    while (running && cap.read(frame)) {
        ++idx;
        GreenDetector::HSVRange hsv;
        if (gui_ok) {
            hsv.h_low = h_low; hsv.h_high = h_high;
            hsv.s_low = s_low; hsv.s_high = s_high;
            hsv.v_low = v_low; hsv.v_high = v_high;
            detector.setMorphKernelSize(morph_k > 0 ? morph_k : 1);
            detector.setMinArea(static_cast<float>(min_area));
        }
        detector.setHSV(hsv);
        auto result = detector.detect(frame, gui_ok);
        if (result.found)
            std::cout << "TARGET: cx=" << result.cx << " cy=" << result.cy
                      << " r=" << result.radius << "\r" << std::flush;
        safeImshow("Green Light Tracker", frame);
        if (safeWaitKey(30) == 27) break;
    }
    std::cout << "\n处理完成，共 " << idx << " 帧\n";
    cap.release(); safeDestroyAll();
    return 0;
}

// ====================== 入口 ======================
int main(int argc, char** argv) {
    std::signal(SIGINT, sigHandler);
    std::signal(SIGTERM, sigHandler);
    probeGUI();

    std::cout << "===== RM 制导飞镖 · 绿光视觉识别 =====\n"
              << "  " << argv[0] << "                 # 摄像头\n"
              << "  " << argv[0] << " /dev/ttyS1       # 摄像头 + MAVLink 串口\n"
              << "  " << argv[0] << " --image <path>    # 图片测试\n"
              << "  " << argv[0] << " --video <path>    # 视频测试\n"
              << "================================\n";

    if (argc == 1)
        return runCamera(0, nullptr);
    else if (argc == 2 && strncmp(argv[1], "/dev/", 5) == 0)
        return runCamera(0, argv[1]);
    else if (argc == 2 && argv[1][0] >= '0' && argv[1][0] <= '9')
        return runCamera(std::atoi(argv[1]), nullptr);
    else if (argc == 3 && std::strcmp(argv[1], "--image") == 0)
        return runImage(argv[2]);
    else if (argc == 3 && std::strcmp(argv[1], "--video") == 0)
        return runVideo(argv[2]);
    else {
        std::cerr << "未知参数\n";
        return -1;
    }
}
