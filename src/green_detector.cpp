#include "green_detector.h"

#include <vector>
#include <algorithm>
#include <cstdio>

GreenDetector::Result GreenDetector::detect(cv::Mat& frame, bool debug) {
    Result res;

    if (frame.empty())
        return res;

    // ---- 1. BGR → HSV ----
    cv::cvtColor(frame, hsv_img_, cv::COLOR_BGR2HSV);

    // ---- 2. 阈值提取绿色区域 ----
    cv::Scalar low(hsv_.h_low, hsv_.s_low, hsv_.v_low);
    cv::Scalar high(hsv_.h_high, hsv_.s_high, hsv_.v_high);
    cv::inRange(hsv_img_, low, high, mask_);

    // ---- 3. 形态学开运算去噪 ----
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                               cv::Size(morph_k_, morph_k_));
    cv::morphologyEx(mask_, morphed_, cv::MORPH_OPEN, kernel);

    // ---- 4. 轮廓检测 ----
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(morphed_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // ---- 5. 分析 ----
    if (!contours.empty()) {
        auto best = std::max_element(contours.begin(), contours.end(),
            [](const auto& a, const auto& b) {
                return cv::contourArea(a) < cv::contourArea(b);
            });

        float area = static_cast<float>(cv::contourArea(*best));
        if (area >= min_area_) {
            // ---- 6. 最小包围圆 + 质心 ----
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(*best, center, radius);

            cv::Moments m = cv::moments(*best);
            if (m.m00 > 0) {
                res.cx = static_cast<float>(m.m10 / m.m00);
                res.cy = static_cast<float>(m.m01 / m.m00);
            } else {
                res.cx = center.x;
                res.cy = center.y;
            }
            res.center = cv::Point2f(res.cx, res.cy);
            res.radius = radius;
            res.area   = area;
            res.found  = true;

            // ---- 7a. 标注: 检测到目标 ----
            if (debug) {
                cv::drawContours(frame, contours,
                                 static_cast<int>(best - contours.begin()),
                                 cv::Scalar(0, 255, 0), 2);
                cv::circle(frame, center, static_cast<int>(radius),
                           cv::Scalar(0, 255, 255), 2);
                cv::drawMarker(frame, res.center, cv::Scalar(0, 0, 255),
                               cv::MARKER_CROSS, 15, 2);
                char buf[64];
                std::snprintf(buf, sizeof(buf), "(%.0f, %.0f) r=%.0f",
                             res.cx, res.cy, radius);
                cv::putText(frame, buf,
                            cv::Point(static_cast<int>(center.x) + 20,
                                      static_cast<int>(center.y) - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            }
        }
    }

    // ---- 7b. 未检测到目标时显示状态 ----
    if (debug && !res.found) {
        cv::putText(frame, "No Target", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    }

    return res;
}
