#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

/**
 * @brief 绿光检测器 —— HSV 色彩空间阈值法
 *
 * 用于识别 RM 机甲大师制导飞镖的绿色引导光源。
 * 检测流程：BGR → HSV → inRange(绿光阈值) → 形态学去噪 →
 *           轮廓查找 → 取最大轮廓 → 计算质心 + 包围圆
 */
class GreenDetector {
public:
    struct Result {
        bool   found = false;       // 是否检测到目标
        float  cx    = 0.0f;        // 质心 x（图像坐标，像素）
        float  cy    = 0.0f;        // 质心 y
        float  radius = 0.0f;       // 包围圆半径（像素）
        float  area   = 0.0f;       // 轮廓面积
        cv::Point2f center;         // 质心（OpenCV 格式）
    };

    // ---------- HSV 阈值（可运行时调整）----------
    struct HSVRange {
        int h_low  = 35;    // 色相下限 (0-180, OpenCV 范围)
        int h_high = 85;    // 色相上限
        int s_low  = 80;    // 饱和度下限
        int s_high = 255;   // 饱和度上限
        int v_low  = 80;    // 亮度下限
        int v_high = 255;   // 亮度上限
    };

    GreenDetector() = default;

    /** 设置 HSV 阈值 */
    void setHSV(const HSVRange& r) { hsv_ = r; }
    const HSVRange& hsv() const { return hsv_; }

    /** 形态学核大小（开运算去噪） */
    void setMorphKernelSize(int k) { morph_k_ = k; }

    /** 最小轮廓面积过滤（忽略噪点） */
    void setMinArea(float a) { min_area_ = a; }

    /**
     * @brief 对一帧图像进行绿光检测
     * @param frame  输入 BGR 图像
     * @param debug  是否绘制调试标记（轮廓 + 质心）
     * @return 检测结果
     */
    Result detect(cv::Mat& frame, bool debug = false);  // frame 非 const：debug 模式会绘制标注

    /** 获取调试用的 mask 图像（二值图） */
    const cv::Mat& debugMask() const { return mask_; }

private:
    HSVRange hsv_;
    int      morph_k_  = 3;        // 形态学核大小
    float    min_area_ = 50.0f;    // 最小轮廓面积 (px²)

    cv::Mat  hsv_img_, mask_, morphed_;
};
