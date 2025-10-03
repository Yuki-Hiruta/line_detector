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
        // カメラ画像のサブスクライブ（例: RealSenseのカラー画像）
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10,
            std::bind(&LineDetectorNode::imageCallback, this, std::placeholders::_1));

        // 検出結果画像のパブリッシュ
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/line_image", 10);
        // 重心座標のパブリッシュ
        cog_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/line_cog", 10);
        // 重心検出有無のパブリッシュ
        is_cog_pub_ = this->create_publisher<std_msgs::msg::Bool>("/is_line_cog", 10);
        // Twist指令値のパブリッシュ
        twist_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/cmd_vel", 10);

        // パラメータ宣言（PIDゲインやHSVしきい値など）
        this->declare_parameter("Kp_x", 0.0f);
        this->declare_parameter("Kp_omega", 0.5f);
        this->declare_parameter("vy", 0.0f);
        this->declare_parameter("lower_h", 0);
        this->declare_parameter("lower_s", 0);
        this->declare_parameter("lower_v", 0);
        this->declare_parameter("upper_h", 0);
        this->declare_parameter("upper_s", 0);
        this->declare_parameter("upper_v", 0);

        // パラメータ取得
        this->get_parameter("Kp_x", Kp_x);
        this->get_parameter("Kp_omega", Kp_omega);
        this->get_parameter("vy", vy); 
        this->get_parameter("lower_h", lower_h);
        this->get_parameter("lower_s", lower_s);
        this->get_parameter("lower_v", lower_v);
        this->get_parameter("upper_h", lower_h); // ←本来はupper_hに格納すべき
        this->get_parameter("upper_s", lower_s); // ←本来はupper_sに格納すべき
        this->get_parameter("upper_v", lower_v); // ←本来はupper_vに格納すべき
    }

private:
    // 重心からTwist指令値を計算しpublish
    void calculateTwist(int cx) {
        // 画面中心からのx方向偏差
        error_x = cx; // 画面中心x座標が320の場合

        // vx = Kp_x * error_x; // 必要に応じて有効化
        // vy = 0.5;           // 必要に応じて有効化
        omega = Kp_omega * error_x;

        // 角速度の上限・下限制限（5度/秒まで）
        if(omega > 5*M_PI/180) omega = 5*M_PI/180;
        if(omega < -5*M_PI/180) omega = -5*M_PI/180;

        // Twist指令値をpublish
        auto twist_msg = std_msgs::msg::Float32MultiArray();
        twist_msg.data.push_back(vx);
        twist_msg.data.push_back(vy);
        twist_msg.data.push_back(omega);
        twist_pub_->publish(twist_msg);
    }

    // 画像受信時のコールバック
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // ROS Image → OpenCV Mat (BGR形式)へ変換
            cv::Mat bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // BGR → HSV色空間へ変換
            cv::Mat hsv;
            cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

            // 画像サイズ取得
            int h = hsv.rows;
            int w = hsv.cols;  

            // 上半分（2/3）を黒塗りしてノイズ除去
            cv::rectangle(hsv, cv::Point(0, 0), cv::Point(w, h *2/ 3), cv::Scalar(0, 0, 0), cv::FILLED);

            // HSVのしきい値設定
            cv::Scalar lower(lower_h, lower_s, lower_v);   // H, S, V の下限
            cv::Scalar upper(upper_h, upper_s, upper_v);   // H, S, V の上限

            // 指定範囲の画素をマスク化
            cv::Mat mask;
            cv::inRange(hsv, lower, upper, mask);

            // 元画像とマスクを合成して色抽出画像を作成
            cv::Mat extracted;
            cv::bitwise_and(bgr, bgr, extracted, mask);

            // 輪郭検出
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // 輪郭が見つからない場合
            if (contours.empty()) {
                auto message_is_cog = std_msgs::msg::Bool();
                message_is_cog.data = false;
                is_cog_pub_->publish(message_is_cog);
                RCLCPP_INFO(this->get_logger(), "No contours detected");
                return;
            }
            // 輪郭が見つかった場合
            else {
                auto message_is_cog = std_msgs::msg::Bool();
                message_is_cog.data = true;
                is_cog_pub_->publish(message_is_cog);
            }

            // 最大面積の輪郭を探索
            size_t maxIdx = 0;
            double maxArea = 0.0;
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

            // 重心を赤丸で描画
            cv::circle(bgr, moment_center, 5, cv::Scalar(0, 0, 255), -1);
            cv::circle(extracted, moment_center, 5, cv::Scalar(0, 0, 255), -1);

            // デバッグ用に重心座標を表示
            RCLCPP_INFO(this->get_logger(), "Line detected at (x, y) = (%d, %d)", cx - 320, cy - 240);

            // 輪郭を緑色で描画
            cv::drawContours(bgr, contours, -1, cv::Scalar(0,255,0), 2);

            // 画像をウィンドウ表示（デバッグ用）
            cv::imshow("Original", bgr);
            // cv::imshow("Mask", mask);
            cv::imshow("Extracted", extracted);
            cv::waitKey(1);

            // 重心座標をpublish
            auto messege_cog = std_msgs::msg::Float32MultiArray();
            messege_cog.data.push_back(cx -320);
            messege_cog.data.push_back(cy - 240);  
            cog_pub_->publish(messege_cog);

            // Twist指令値を計算・publish
            calculateTwist(cx - 320);

            // もし変数に保持したいなら extracted をクラスメンバに保存
            // latest_extracted_ = extracted.clone();
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    // --- メンバ変数 ---
    float vx = 0;    // 前進速度
    float vy = 0;    // 横移動速度
    float omega = 0; // 角速度

    int error_x = 0; // 画面中心からのx方向誤差
    float Kp_x = 0.0f;     // x方向比例ゲイン
    float Kp_omega = 0.5;  // 角速度比例ゲイン

    bool is_cog = false;   // 重心検出フラグ

    int lower_h, lower_s, lower_v; // HSV下限
    int upper_h, upper_s, upper_v; // HSV上限

    // ROS2通信関連
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cog_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_cog_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr twist_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
