#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class LineDetectorNode : public rclcpp::Node
{
public:
    LineDetectorNode()
    : Node("line_detector")
    {
        // RealSenseなどからのカメラ映像をsub
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10,
            std::bind(&LineDetectorNode::imageCallback, this, std::placeholders::_1));

        // 検出結果をpublishするトピック
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/line_image", 10);
        cog_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/line_cog", 10);
        is_cog_pub_ = this->create_publisher<std_msgs::msg::Bool>("/is_line_cog", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // 5フレームに1回だけ処理する
            static int frame_count = 0;
            frame_count++;
            if(frame_count % 5 != 0) return; 

            // ROS Image → OpenCV Mat (BGR形式)
            cv::Mat bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // cv::Mat hsv = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // BGR → HSV
            cv::Mat hsv;
            cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
            // 画像サイズ取得
            int h = hsv.rows;
            int w = hsv.cols;  
            // 上半分を黒塗り
            cv::rectangle(hsv, cv::Point(0, 0), cv::Point(w, h *2/ 3), cv::Scalar(0, 0, 0), cv::FILLED);

            // 抽出したい色の範囲 
            cv::Scalar lower(0, 100, 150);   // H, S, V の下限
            cv::Scalar upper(40, 255, 255);  // H, S, V の上限

            // 範囲内の画素をマスク化
            cv::Mat mask;
            cv::inRange(hsv, lower, upper, mask);

            // 元画像とマスクを合成して色抽出画像を作成
            cv::Mat extracted;
            cv::bitwise_and(bgr, bgr, extracted, mask);

            // 輪郭検出
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // 最大面積の輪郭を探す
            size_t maxIdx = 0;
            double maxArea = 0.0;

            // 輪郭が見つからない場合の処理
            if (contours.empty()) {
                auto message_is_cog = std_msgs::msg::Bool();
                message_is_cog.data = false;
                is_cog_pub_->publish(message_is_cog);
                RCLCPP_INFO(this->get_logger(), "No contours detected");
                return;
            }

            // 輪郭が見つかった場合の処理
            else {
                auto message_is_cog = std_msgs::msg::Bool();
                message_is_cog.data = true;
                is_cog_pub_->publish(message_is_cog);
            }

            // 最大面積の輪郭を探す
            for (size_t i = 0; i < contours.size(); ++i) {
                double area = cv::contourArea(contours[i]);
                if (area > maxArea) {
                    maxArea = area;
                    maxIdx = i;
                }
            }

            // 最大輪郭の重心を計算
            cv::Moments m = cv::moments(contours[maxIdx]);
            if (m.m00 == 0) return;
            int cx = static_cast<int>(m.m10 / m.m00);
            int cy = static_cast<int>(m.m01 / m.m00);
            const cv::Point2f moment_center = cv::Point2f(cx,cy);
            // cv::circle(bgr, moment_center, 5, cv::Scalar(0, 0, 255), -1);
            cv::circle(extracted, moment_center, 5, cv::Scalar(0, 0, 255), -1);
            RCLCPP_INFO(this->get_logger(), "Line detected at (x, y) = (%d, %d)", cx - 320, cy - 240);

            // // 表示（デバッグ用）
            // cv::imshow("Original", bgr);
            // cv::imshow("Mask", mask);
            cv::imshow("Extracted", extracted);
            cv::waitKey(1);

            auto messege_cog = std_msgs::msg::Float32MultiArray();
            messege_cog.data.push_back(cx -320);
            messege_cog.data.push_back(cy - 240);  
            cog_pub_->publish(messege_cog);

            // もし変数に保持したいなら extracted をクラスメンバに保存
            // latest_extracted_ = extracted.clone();
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    bool is_cog = false;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cog_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_cog_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
