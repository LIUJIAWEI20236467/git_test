

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tdt_interface/msg/receive_data.hpp>
#include <tdt_interface/msg/send_data.hpp>
#include <cmath>
#include <queue>
#include <map>
#include <cmath>
#include <vector>
#include <iostream>
#include <queue>
#include <unordered_map>
#define PI 3.14159265358979323846

using namespace cv;
using namespace std;
//using namespace Eigen;
struct nodes
{
    float x, y;
    float g, h;// g:从起点到该节点的距离，h:从该节点到终点的距离
    float f()const{return g+h;}//总成本
    bool operator>(const nodes& other)const{return f()>other.f();}
    //如果当前节点的成本>other节点的成本，返回true  
};
class LightBar {
public:
    RotatedRect rect; // 灯条的旋转矩形
    bool used; // 灯条是否被使用

    // 构造函数
    LightBar(const RotatedRect& r) : rect(r) ,used(false){}

    // 判断灯条是否被使用
    bool isused() const {
        return used;
    }
    // 设置灯条是否被使用
    void setused(bool u) {
        used = u;
    }

    // 绘制灯条
    void draw(Mat& image) const {
        Point2f vertices[4];
        rect.points(vertices);
        for (int i = 0; i < 4; i++) {
            line(image, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 1.5);
        }
    }

    // 获取灯条中心点
    Point2f center() const {
        return rect.center;
    }
    Point2f top()const {
        vector<Point2f> corners(4);
        rect.points(&corners[0]);

        // 对角点按Y坐标从小到大排序，确保上下的顺序
        for (int i = 0; i < corners.size(); ++i) {
            for (int j = 0; j < corners.size() - i - 1; ++j) 
            {
                if (corners[j].y > corners[j + 1].y || (corners[j].y == corners[j + 1].y && corners[j].x > corners[j + 1].x))
                {
                    swap(corners[j], corners[j + 1]);
                }
            }
        }

        auto top = (corners[0] + corners[1]) / 2;
        return top;
    }
    Point2f bottom()const {
        vector<Point2f> corners(4);
        rect.points(&corners[0]);
         // 对角点按Y坐标从小到大排序，确保上下的顺序
        for (int i = 0; i < corners.size(); ++i) {
            for (int j = 0; j < corners.size() - i - 1; ++j) {
                if (corners[j].y > corners[j + 1].y || 
                   (corners[j].y == corners[j + 1].y && corners[j].x > corners[j + 1].x)) {
                    std::swap(corners[j], corners[j + 1]);
                }
            }
        }

        auto bottom = (corners[2] + corners[3]) / 2;
        return bottom;
    }
    vector<Point2f> points3()const {
        return {top(),bottom(),top()};
    }//获取灯条上中下三个点的坐标

    float angle() const {
        vector<Point2f> corners(4);
        rect.points(&corners[0]);

        // 对角点按Y坐标从小到大排序，确保上下的顺序
        for (int i = 0; i < corners.size(); ++i) {
            for (int j = 0; j < corners.size() - i - 1; ++j) {
                if (corners[j].y > corners[j + 1].y || 
                   (corners[j].y == corners[j + 1].y && corners[j].x > corners[j + 1].x)) {
                    std::swap(corners[j], corners[j + 1]);
                }
            }
        }

        auto top = (corners[0] + corners[1]) / 2;
        auto bottom = (corners[2] + corners[3]) / 2;
        auto top2bottom = bottom - top;
        return std::atan2(top2bottom.y, top2bottom.x);
    }
    //10.9郭嘉
    float angle_gj() {
            if(rect.size.width > rect.size.height){return rect.angle; }
            else{return rect.angle+90.0;}
    }

    float angle_tj() {
        return abs(LightBar::angle()- CV_PI / 2);
    }
    float angle_raw() const {
        return rect.angle;
    }
    // 获取灯条的高度
    float height() const {
        return max(rect.size.width, rect.size.height);
    }

    // 获取灯条的宽度
    float width() const {
        return min(rect.size.width, rect.size.height);
    }
};
class Armor {
public:
    LightBar leftBar;  // 左灯条
    LightBar rightBar; // 右灯条
    //RotatedRect armorRect; // 装甲板的旋转矩形
    Point2f fourpoints[4]; // 装甲板四个顶点
    Point2f armorCenter; // 装甲板中心点

    // 构造函数，通过两个灯条构造装甲板
    Armor(const LightBar& lb, const LightBar& rb)
        : leftBar(lb), rightBar(rb) {
        calculateArmor();
    }

    // 计算装甲板的位置、尺寸和角度
    void calculateArmor() 
    {
        armorCenter = (leftBar.center() + rightBar.center()) / 2;
        float armorAngle = (leftBar.angle_gj() + rightBar.angle_gj()) / 2;

        fourpoints[0] = Point2f(leftBar.top().x, leftBar.top().y);
        fourpoints[1] = Point2f(leftBar.bottom().x, leftBar.bottom().y);
        fourpoints[2] = Point2f(rightBar.bottom().x, rightBar.bottom().y);   
        fourpoints[3] = Point2f(rightBar.top().x, rightBar.top().y);
    }
    
    double pnp() //进行solvepnp姿态解算
    {
        vector<Point2f>points6;//2D灯条图像点，相机坐标系的像素坐标
        vector<Point2f>left=leftBar.points3();
        vector<Point2f>right=rightBar.points3();
        points6.insert(points6.end(),left.begin(),left.end());
        points6.insert(points6.end(),right.begin(),right.end());//将2个2D灯条图像点转换到相机坐标系的像素坐标

        vector<Point3f>points3d;//3D灯条坐标
        points3d.push_back(Point3f(-67,27.5,0));
        points3d.push_back(Point3f(-67,0,0));
        points3d.push_back(Point3f(-67,-27.5,0)); //左下角为坐标系原点   
        points3d.push_back(Point3f(67,27.5,0));
        points3d.push_back(Point3f(67,0,0));
        points3d.push_back(Point3f(67,-27.5,0));

        if (points3d.size() != points6.size())//判断输入的2D点和3D点的数量是否一致
        {
            cerr << "Error: Number of 2D points does not match number of 3D points." << endl;
            return 0;
        }
        //else cout << "solvepnp" << endl;

         // 相机内参矩阵
        // Mat camera_matrix = (Mat_<double>(3, 3) << 
        // 623.5383, 0.0, 640,
        // 0.0, 1108.513, 360,
        // 0.0, 0.0, 1.0);
        Mat camera_matrix = (Mat_<double>(3, 3) << 
            1.7774091341308808e+03, 0.0, 7.1075979428865026e+02,
            0.0, 1.7754170626354828e+03, 5.3472407285624729e+02,
            0.0, 0.0, 1.0);

        //畸变系数
        Mat dist_coeffs = (Mat_<double>(1, 5) << 
        -5.6313426428564950e-01, 1.8301501710641366e-01, 
        1.9661478907901904e-03, 9.6259122849674621e-04, 
        5.6883803390679100e-01);
        //Mat dist_coeffs = Mat::zeros(1, 5, CV_64F); // 全为零的畸变矩阵


        Mat rvec, tvec; // 存储旋转向量和位移向量
        bool success = solvePnP(points3d, points6, camera_matrix, dist_coeffs, rvec, tvec);
        if(!success){cerr << "error:solvepnp failed" << endl;return 0;}

        double distance = norm(tvec);
        //cout<<"距离 to camera center:"<<distance<<endl;
        Mat image = Mat::zeros(480, 640, CV_8UC3);

        
        return distance;
        
    }
    Point3d erdto3d(Point2d point,double z)//将2D点转换到3D点
    {
        // 相机内参矩阵
        Mat camera_matrix = (Mat_<double>(3, 3) << 
            1.7774091341308808e+03, 0.0, 7.1075979428865026e+02,
            0.0, 1.7754170626354828e+03, 5.3472407285624729e+02,
            0.0, 0.0, 1.0);

        // 畸变系数
        Mat dist_coeffs = (Mat_<double>(1, 5) << 
            -5.6313426428564950e-01, 1.8301501710641366e-01, 
            1.9661478907901904e-03, 9.6259122849674621e-04, 
            5.6883803390679100e-01);

        vector<Point2d> undistortedpoints;// 存储去畸变的2D点
        vector<Point2d> distortedpoints={point};// 存储畸变的2D点
        undistortPoints(distortedpoints, undistortedpoints, camera_matrix, dist_coeffs);
        Point2d undistortedpoint=undistortedpoints[0];

        double x = (undistortedpoint.x * z);// camera_matrix.at<double>(0, 0);
        double y = (undistortedpoint.y * z);// camera_matrix.at<double>(1, 1);
        cout<<"x:"<<x<<" y:"<<y<<" z:"<<z<<endl;
        return Point3d(x,y,z);
    }

    double flytime()
    {
        double speed = 30000.0; // 速度
        auto fly_time = pnp() / speed; // 飞行时间
        //cout << "飞行时间：" << fly_time << endl;
        return fly_time;
    }

    // 绘制装甲板
    void draw(Mat& image) const 
    {
        Point2f vertices[4];
        for (int i = 0; i < 4; i++) 
        {
            line(image, fourpoints[i], fourpoints[(i + 1) % 4], Scalar(255, 255, 0), 2);
        }
        circle(image, armorCenter, 4, Scalar(0, 255, 255), -1); // 绘制中心点
    }

    void draw_result(Mat& image) const 
    {
        circle(image, armorCenter, 11, Scalar(0, 0, 255), 2); // 绘制中心点
    }
    
    bool armor_ratio()
    {
        double chang=(norm(leftBar.top()-rightBar.top())+norm(leftBar.bottom()-rightBar.bottom()))/2;
        double duan=(norm(leftBar.top()-leftBar.bottom())+norm(rightBar.top()-rightBar.bottom()))/2;
        double ratio=chang/duan;
        if(ratio<2.8&&ratio>1.5){return  true;}
        else{return false;}
    }
};
class KalmanFilter_mine
{
public:
    KalmanFilter_mine(double dt)
    {
        // 状态转移矩阵
        F = (Mat_<double>(4, 4) << 
            1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1);

        // 观测矩阵
        H = (Mat_<double>(2, 4) << 
            1, 0, 0, 0,
            0, 1, 0, 0);

        // 过程噪声协方差和测量噪声协方差
        Q = Mat::eye(4, 4, CV_64F) * 1.5;
        R = Mat::eye(2, 2, CV_64F) * 1;

        // 初始状态
        reset();
    }

    // 预测步骤
    void predict()
    {
        X = F * X;
        P = F * P * F.t() + Q;
    }

    // 更新步骤
    void update(const Mat &measurement)
    {
        predict(); // 先进行预测
        Mat K = P * H.t() * (H * P * H.t() + R).inv();
        X = X + K * (measurement - H * X);
        P = (Mat::eye(4, 4, CV_64F) - K * H) * P;
    }

    // 重置卡尔曼滤波器
    void reset(double x = 0, double y = 0, double vx = 0, double vy = 0)
    {
        X = (Mat_<double>(4, 1) << x, y, vx, vy);
        P = Mat::eye(4, 4, CV_64F);
    }

    Mat get_X() const { return X; }

    void set_dt(double input_dt) { dt = input_dt; }

    void set_v(double vx, double vy)
    {
        X.at<double>(2) = vx;
        X.at<double>(3) = vy;
    }

private:
    Mat F, H, Q, R, X, P;
    double dt;
};

class AutoShoot : public rclcpp::Node
{
public:
    AutoShoot()
        : Node("auto_shoot_node"),kf(30)
    {
        // 订阅相机图像
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_image", 10,
            std::bind(&AutoShoot::imageCallback, this, std::placeholders::_1));

        // 订阅云台角度
        receive_data_sub_ = this->create_subscription<tdt_interface::msg::ReceiveData>(
            "/real_angles", 10,
            std::bind(&AutoShoot::receiveCallback, this, std::placeholders::_1));

        // 订阅栅格地图
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            std::bind(&AutoShoot::mapCallback, this, std::placeholders::_1));

        // 订阅当前机器人位姿
        position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/position", 10,
            std::bind(&AutoShoot::positionCallback, this, std::placeholders::_1));

        // 订阅当前真实速度
        real_speed_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/real_speed", 10,
            std::bind(&AutoShoot::realSpeedCallback, this, std::placeholders::_1));

        // 订阅目标点位姿
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&AutoShoot::goalPoseCallback, this, std::placeholders::_1));

        // 发布目标云台角度
        send_data_pub_ = this->create_publisher<tdt_interface::msg::SendData>("/target_angles", 10);

        // 发布目标速度
        speed_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/target_speed", 10);

        // 发布比赛开始信号
        game_start_pub_ = this->create_publisher<std_msgs::msg::Bool>("/game_start", 10);

        publishGameStartSignal();
        int shoot_mark = 0;
        
    }

private:
    void publishGameStartSignal() // 发布比赛开始信号
    {
        auto msg = std::make_shared<std_msgs::msg::Bool>();
        msg->data = true;
        game_start_pub_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Game start");
    }
    //图像处理和自瞄（发布yaw，pitch与射击）
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) // 图像回调函数
    {   //if(1)
        if(is_shooting)
        {
            auto target_speed_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
            linear_speed_y = 0;
            linear_speed_x =0;
            target_speed_msg->twist.linear.y = linear_speed_y;
            target_speed_msg->twist.linear.x = linear_speed_x;
            speed_pub_->publish(*target_speed_msg);//发布速度

            is_navigating=false;
            auto send_data_msg = std::make_shared<tdt_interface::msg::SendData>();
            cv::Mat frame;
            std::vector<uint8_t> jpeg_data(msg->data.begin(), msg->data.end());
            frame = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
        
            if (frame.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to decode image.");
                return;
            }

            /********************处理你的图像*********************/
            Mat gray;cvtColor(frame, gray, COLOR_BGR2HSV);
            Scalar red_jin_lower(0, 70, 170); Scalar red_jin_upper(30, 180, 255);
            Scalar red_yuan_lower(139, 70, 170); Scalar red_yuan_upper(179, 189, 255);
            Scalar red_mid_lower(0, 140, 100); Scalar red_mid_upper(15, 179, 220);
            Scalar red_all_lower(150, 140, 100); Scalar red_all_upper(179, 179, 220);
            Mat mask1,mask2,mask3,mask,mask4; 
            inRange(gray, red_jin_lower, red_jin_upper, mask1); 
            inRange(gray, red_yuan_lower, red_yuan_upper, mask2);
            inRange(gray, red_mid_lower, red_mid_upper, mask3);
            inRange(gray, red_all_lower, red_all_upper, mask4);
            mask = mask1 | mask2 | mask3 | mask4; // 合并两个红色
            vector<vector<Point>> contours; Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
            Mat canny; Canny(mask2, canny, 50, 150);
            Mat dilate_frame; dilate(canny, dilate_frame, kernel); // 膨胀操作
            findContours(dilate_frame, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE); // 轮廓检测

            vector<LightBar> lightBars; // 识别出来的灯条
            if(contours.size() > 0)
            {
                light_mark = 0;
                //cout << contours.size() << endl;
                for (const auto &contour : contours)
                {
                    if (contourArea(contour) > 10&&contourArea(contour)<800) 
                    {
                        RotatedRect rotatedRect = minAreaRect(contour); vector<Point2f> corners(4);            
                        auto ratio = max(rotatedRect.size.height, rotatedRect.size.width) / min(rotatedRect.size.height, rotatedRect.size.width);

                        if (ratio>2.1 && LightBar(rotatedRect).angle_gj()>60 && LightBar(rotatedRect).angle_gj()<120)
                        {
                            lightBars.push_back(LightBar(rotatedRect));
                            //LightBar(rotatedRect).draw(frame); // 绘制灯条
                        }
                    }
                }

                if(lightBars.size()>0)
                {
                    for (int i = 0; i < lightBars.size(); i++)//给灯条排序
                    {
                        for (int j = i + 1; j < lightBars.size(); j++) 
                        {
                            if (lightBars[i].center().x > lightBars[j].center().x) 
                            {
                                swap(lightBars[i], lightBars[j]); 
                            }
                        }
                    }
                } 
                else  
                {
                    yaw = yaw -0.5;//if(pitch<0){pitch+=0.1;}        
                }

                vector<Armor> armors; // 装甲板集合
                int n = lightBars.size();
                for (int i = 0; i < n; i++) 
                {
                    if (lightBars[i].isused()) continue; // 跳过已使用的灯条
                    for (int j = i + 1; j < n; j++)
                    {
                        if (lightBars[j].isused()) continue; // 跳过已使用的灯条

                        Point2f pointA = lightBars[i].center(); Point2f pointB = lightBars[j].center();
                        auto arm_center = (pointA + pointB) / 2;
                        auto arm_width = (lightBars[i].width() + lightBars[j].width()) * 2.5;
                        auto arm_height = (lightBars[i].height() + lightBars[j].height()) * 0.9;
                        auto length_diff = abs(lightBars[i].height()/lightBars[j].height());
                        auto angle_diff = abs(lightBars[i].angle_gj() - lightBars[j].angle_gj());
                        auto angle_center =abs(atan2(pointB.y - pointA.y, pointB.x - pointA.x))*180/PI;
                        if (angle_diff < 30 && angle_center<30)
                        {
                            if (length_diff < 1.8 && length_diff > 0.3)
                            {
                                RotatedRect armorRect(arm_center, Size2f(arm_width, arm_height), (lightBars[i].angle() + lightBars[j].angle()) / 2);
                                Armor armor(lightBars[i], lightBars[j]); // 构造装甲板
                                if (armor.armor_ratio()) {armors.push_back(armor);} else {break;}
                                lightBars[i].setused(true); lightBars[j].setused(true);//标记为已使用灯条
                                //armor.draw(frame); // 绘制装甲板
                            }
                        }
                    }
                }

                if(armors.size() > 0)
                {
                    int index = 0;
                    double min_distance = 1000000;

                    for (int i = 0; i < armors.size(); i++)
                    {
                        if (armors[i].pnp () <min_distance) 
                        {
                            min_distance=armors[i].pnp();
                            index = i;
                            if(target_center.size()>=2)//只有两个点
                            {
                                target_center.erase(target_center.begin());
                            }
                            target_center.push_back(armors[i].armorCenter);

                        }
                    }
                    Point2f observed_center = armors[index].armorCenter;
                    Mat measurement = (Mat_<double>(2, 1) << observed_center.x, observed_center.y);

                    // 预测下一个位置
                    double time = 30; // 假设下一帧时间
                    double pred_x = kf.get_X().at<double>(0) + kf.get_X().at<double>(2) * time;
                    double pred_y = kf.get_X().at<double>(1) + kf.get_X().at<double>(3) * time;
                    //Point2d pred_point(pred_x, pred_y);
                    //auto targetX = pred_x; auto cameraX = frame.cols / 2;
                    //auto targetY = pred_y; auto cameraY = frame.rows / 2;

                    auto targetX = armors[index].armorCenter.x; auto cameraX = frame.cols / 2;
                    auto targetY = armors[index].armorCenter.y; auto cameraY = frame.rows / 2;
                    auto diff_x = targetX - cameraX; auto diff_y = cameraY - targetY;
                    auto yaw_d = diff_x / frame.cols * 90;
                    auto pitch_d = diff_y / frame.rows * 60;

                    circle(frame, armors[index].armorCenter, 5, Scalar(0, 0, 255), 2);//绘制出目标点

                    //if(yaw_d > 5){yaw_d = 5;} if(yaw_d < -5){yaw_d = -5;}
                    if(pitch_d > 2){pitch_d = 2;} if(pitch_d < -2){pitch_d = -2;}
                    yaw = yaw + yaw_d*1.25; pitch = pitch + pitch_d;
                }   
                 else 
                //if(target_center.size()==2 && lightBars.size()!=0)
                //     {
                //         auto diff_center_dist = target_center[1].x - target_center[0].x;
                //         if(diff_center_dist<30 && diff_center_dist>-30 && diff_center_dist!=0)
                //         {
                //             yaw = yaw + diff_center_dist/10;
                //             pitch = pitch + diff_center_dist/45;
                //         }
                //         else if(diff_center_dist>30&&diff_center_dist<50&&diff_center_dist>-30&&diff_center_dist!=0)
                //                 {
                //                     yaw = yaw + diff_center_dist/5;
                //                     pitch = pitch + diff_center_dist/30;
                //                 }
                //                 else
                //                 { 
                //                     yaw = yaw + 2; /*if(pitch<0) {pitch+=0.1;} */
                //                 }
                        
                //     }
                //     else   
                //     { 
                //         yaw = yaw + 2; if(pitch>17) {pitch-=0.5;}if (pitch < -17){pitch += 0.5;}
                //     }
                if(armors.size() == 0 && lightBars.size() != 0)
                {
                    auto targetX = lightBars[0].center().x; auto cameraX = frame.cols / 2;
                    auto targetY = lightBars[0].center().y; auto cameraY = frame.rows / 2;
                    auto diff_x = targetX - cameraX; auto diff_y = cameraY - targetY;
                    auto yaw_d = diff_x / frame.cols * 90; auto pitch_d = diff_y / frame.rows * 45;

                    if(yaw_d > 5){yaw_d = 5;} if(yaw_d < -5){yaw_d = -5;}
                    if(pitch_d > 2){pitch_d = 2;} if(pitch_d < -2){pitch_d = -2;}
                    yaw = yaw + yaw_d*1.2; pitch = pitch + pitch_d;
                }
                    send_data_msg->if_shoot = true;
            }
            else 
            {
                yaw = yaw + 5;
                light_mark++;
                 if (pitch < -17)
                 {
                     pitch += 0.5;
                 }
                 if(pitch>17)
                 {
                     pitch -= 0.5;
                 }
                send_data_msg->if_shoot = false;
            }

            //circle(frame, Point(frame.cols / 2, frame.rows / 2), 5, Scalar(255, 10, 255), 1); // 绘制出目标点
            //imshow("Camera Image", frame);
            waitKey(1);

            // /********************发布你应该发布的角度**************************/            
            send_data_msg->pitch = pitch;
            send_data_msg->yaw = yaw;
            send_data_pub_->publish(*send_data_msg);
            if(light_mark>=330)
            {
                light_mark=0;
                is_shooting = false;
                is_navigating = true;
            }
        }
    }
    //设置各项速度（发布目标速度，并接收实际速度）
    void realSpeedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        real_linear_speed_x = msg->twist.linear.x;
        real_linear_speed_y = msg->twist.linear.y;
        //RCLCPP_INFO(this->get_logger(), "Real speed: x: %f, y: %f", real_linear_speed_x, real_linear_speed_y);
        //if(0)
        if(is_navigating)//开始导航
        {
            //real_linear_speed_x = msg->twist.linear.x;
            //real_linear_speed_y = msg->twist.linear.y;
            if(robot_x_point==0){return;}
            int a = 0;
            nodes robot;//当前状态
            nodes start;//起始状态
            robot.x = robot_x_point;
            robot.y = robot_y_point;
            
            // if(my_flag==0&&robot_x_point==1&&robot_y_point==1)
            // {
            //     paths.push_back( a_star(1, 1, center_points_path[0].y, center_points_path[0].x, grid)); // 寻找路径
            //     key_points.push_back( get_path_points(paths[0]));//寻找转弯关键点
            // }
            //if()
            if(flag!=index)//说明成功到达目标点
            {
                if(flag!=key_points[my_flag].size()-1)//还未到达最后一个点
                {
                    mark_x=robot.x;
                    mark_y=robot.y;
                    index++;
                    //cout <<key_points[my_flag].size() - 2 - flag<< endl;
                    a = con_move(robot, key_points[my_flag][flag + 1], robot);
                }else 
                {//开启下一段path的导航（my_flag）
                    flag=0;
                    index = 0;
                    is_shooting = true;//到达目标点后开始自瞄
                    is_navigating = false;//停止导航
                    my_flag++;


                    if(my_flag>5){cerr<<"my_flag--超出范围--937"<<endl;return;}
                    // if(real_linear_speed_x==0&&real_linear_speed_y==0) 
                    // {
                    //     paths.push_back( a_star(robot.x, robot.y, center_points_path[my_flag].y, center_points_path[my_flag].x, grid)); // 寻找路径
                    //     key_points.push_back( get_path_points(paths[my_flag]));//寻找转弯关键点
                    // }
                }
            }

            if(flag==index)
            {
                if(flag==0&&my_flag==0)//导航到第一个点
                {start.x = 1.0;start.y = 1.0;}
                else
                {start.x = mark_x;start.y = mark_y;}

              
                if(key_points[my_flag].size() >=0)//调试用
                {
                cout <<key_points[my_flag].size() - 3 - flag<< endl;
                }

                a = con_move(start, key_points[my_flag][flag + 1], robot);
            }
            if(a)flag++;
            if(flag>=key_points[my_flag].size()-4)
            {
                qaz = 1;
            }
        }
    }
    // 云台角度回调（接收真实的yaw与pitch）
    void receiveCallback(const tdt_interface::msg::ReceiveData::SharedPtr msg)
    {
        pitch = msg->pitch;
        yaw = msg->yaw;
        RCLCPP_INFO(this->get_logger(), "Receive data: pitch: %f, yaw: %f", pitch, yaw);
    }
    //地图处理与规划路径
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        /*****************************保存或处理你的地图*****************************/
        int width = msg->info.width;
        int height = msg->info.height;//1250*1250
        cv::Mat map_image(height, width, CV_8UC1);

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                int index = x + y * width;
                int8_t occupancy_value = msg->data[index];
                uint8_t pixel_value = 0;
                if (occupancy_value == 0)
                    pixel_value = 255;
                else if (occupancy_value == 100)
                    pixel_value = 0;
                else
                    pixel_value = 128;

                map_image.at<uint8_t>(y, x) = pixel_value;
            }
        } 

      
        
            grid = convertImageToGrid(map_image);//处理地图
            center_points_path = get_center_points(grid);//根据地图寻找5个目标区域的中心点
        
           
            if(key_points.size()>=5){return;}
            paths.push_back( a_star(1, 1, center_points_path[0].y, center_points_path[0].x, grid)); // 寻找路径
            paths.push_back( a_star(center_points_path[0].y, center_points_path[0].x, center_points_path[1].y, center_points_path[1].x, grid));
            paths.push_back( a_star(center_points_path[1].y, center_points_path[1].x, center_points_path[2].y, center_points_path[2].x, grid));
            paths.push_back( a_star(center_points_path[2].y, center_points_path[2].x, center_points_path[3].y, center_points_path[3].x, grid));
            paths.push_back( a_star(center_points_path[3].y, center_points_path[3].x, center_points_path[4].y, center_points_path[4].x, grid));
            key_points.push_back( get_path_points(paths[0]));//寻找转弯关键点
            key_points.push_back( get_path_points(paths[1]));
            key_points.push_back( get_path_points(paths[2]));
            key_points.push_back( get_path_points(paths[3]));
            key_points.push_back( get_path_points(paths[4]));
        
        //cv::imshow("Occupancy Grid Map", map_image);
        cv::waitKey(1);
    }

    float heuristic(int x1, int y1, int x2, int y2)
    {
        return abs(x1 - x2) + abs(y1 - y2);
    }
    //图像转换函数，将1250处理成数组
    vector<vector<int>> convertImageToGrid(const Mat& image, int step = 25) 
    {
        //imshow("origin_image", image);
        int rows = image.rows;
        int cols = image.cols;
        Mat gray_image;
        if (image.channels() > 1) {
            cv::cvtColor(image, gray_image, COLOR_BGR2GRAY);
        } else {
            gray_image = image; // 如果已经是单通道，直接使用
        }

        // 二值化处理
        Mat binary_image;
        cv::threshold(gray_image, binary_image, 130, 255, THRESH_BINARY_INV); // 黑色为 0，白色为 255
        //imshow("binary_image", binary_image);

        // 使用步长 step 初始化网格大小
        int new_rows = rows / step+1;
        int new_cols = cols / step+1;
        vector<vector<int>> grid(new_rows, vector<int>(new_cols, 0));

        // 每 step 个像素进行一次判断
        for (int i = 0; i <=rows; i += step) 
        {
            for (int j = 0; j <= cols; j += step) 
            {
                //二值化后，黑白颠倒
                // 假设图像为单通道，黑色为 0，白色为 255
                if (binary_image.at<uchar>(i, j) == 0) {
                    grid[i / step][j / step] = 0;
                } else {
                    grid[i / step][j / step] = 1;//颠倒后，白色变为障碍物
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "图像转换成功");
        for (int j = 0; j < grid[50].size(); ++j) {grid[50][j] = 1;}
        return grid;
    }
    //寻找5个目标区域中心点,并根据路程排序
    vector<Point> get_center_points(const vector<vector<int>>& grid) 
    {
        Point zero(1, 1); 
       
        for (int i = 1; i < 50; ++i) 
        {
            for (int j = 1; j < 50; ++j)
            {
                if (grid[i][j] == 0 && 
                    grid[i-1][j] == 0 && grid[i+1][j] == 0 && 
                    grid[i][j-1] == 0 && grid[i][j+1] == 0 &&    
                    grid[i+1][j+1] == 0 && grid[i+1][j-1] == 0 &&
                    grid[i-1][j+1] == 0 && grid[i-1][j-1] == 0 && 
                    grid[i+2][j+2] == 0&&grid[i-2][j-2] == 0&&grid[i-2][j+2] == 0&&grid[i+2][j-2] == 0)
                {
                    rough_region_centers.push_back(Point(j, i));
                    //cout << "rough " << j << "," << i << endl;
                }
            }
        }
       
        if (!rough_region_centers.empty()) 
        {
            // 添加第一个粗糙区域中心点
            //cout <<"!";
            detailed_region_centers.push_back(rough_region_centers[0]);

            for (int i = 1; i < rough_region_centers.size(); ++i)
            {
                    bool is_valid = true;  // 用于检查是否满足条件

                    for (const auto& center : detailed_region_centers) 
                    {
                        double dist_squared = 
                            pow(center.x - rough_region_centers[i].x, 2) +
                            pow(center.y - rough_region_centers[i].y, 2);

                        // 如果与已有中心点的距离小于或等于100，则不添加
                        if (dist_squared <= 36) 
                        {
                            is_valid = false;
                            break; // 不满足条件，退出循环
                        }
                    }

                    // 如果与所有已有中心点的距离都大于100，则添加
                    if (is_valid) 
                    {
                        detailed_region_centers.push_back(rough_region_centers[i]);
                        //cout << "detailed " << rough_region_centers[i].x << " " << rough_region_centers[i].y << endl;
                    }

                    // 如果已经找到了5个点，则退出循环
                    if (detailed_region_centers.size() >= 5) 
                    {
                        break;
                    }
            }
        }   
       
        vector<Point> temp;
        while (!detailed_region_centers.empty()) 
        {
            float min_dist = numeric_limits<float>::max();
            int my_index = -1;

            for (int i = 0; i < detailed_region_centers.size(); ++i) 
            {
                vector<nodes> path = a_star(detailed_region_centers[i].y, detailed_region_centers[i].x, zero.y, zero.x, grid);
                float dist = path.size(); // 假设 path.size() 返回路径长度

                if (dist < min_dist) 
                {
                    min_dist = dist;
                    my_index = i;
                }
            }

            if (my_index != -1) 
            {
                // 将最近点加入排序结果并更新参考点
                temp.push_back(detailed_region_centers[my_index]);
                zero = detailed_region_centers[my_index];

                // 从 detailed_region_centers 中移除该点
                detailed_region_centers.erase(detailed_region_centers.begin() + my_index);
            } else
             {
                // 如果没有找到更近的点，可能是由于一些错误或异常情况，这里简单处理为结束循环
                break;
            }
        }
        return temp;
    }

    vector<nodes> get_neighbors(nodes node,const vector<vector<int>>& grid)
    {
        vector <nodes> neighbors; 
        vector <pair<int,int>> directions = {{0,1},{0,-1},{1,0},{-1,0}};//上下左右
        for(auto direction:directions)
        {
            float x = node.x + direction.first;
            float y = node.y + direction.second;
            if(x>=0&&x<grid.size()&&y>=0&&y<grid[0].size()&&grid[x][y]==0)
            {   
                neighbors.push_back({x, y, 0, 0});
            }
        }
        return neighbors;
    }
    
    vector<nodes> a_star(float start_x, float start_y, float target_x, float target_y, const vector<vector<int>>& grid) 
    {
        auto grid_width = grid[0].size();
        auto grid_height = grid.size();
        
        // 如果目标点是障碍物，直接返回
        if (grid[target_x][target_y] == 1)
        {
            cerr << "目标点是障碍物." << endl;return {};
        } else 

        {
            RCLCPP_INFO(this->get_logger(), "目标点是可通行.");
        }

        // 初始化open set优先队列
        priority_queue<nodes, vector<nodes>, greater<nodes>> open_set;

        // 记录所有访问过的节点及其最小开销
        unordered_map<int, nodes> all_nodes;// 记录所有访问过的节点
        unordered_map<int, int> came_from; // 记录路径

        // 初始化起点
        nodes start = {start_x, start_y, 0, heuristic(start_x, start_y, target_x, target_y)};// 起点的 g 和 h 值
        open_set.push(start);// 将起点加入优先队列
        all_nodes[start_x * grid_width + start_y] = start;// 记录起点

        // 主循环
        while (!open_set.empty()) 
        {
            nodes now = open_set.top();// 取出优先队列中最低开销的节点
            open_set.pop();// 从优先队列中删除该节点

            // 如果已经到达目标节点，开始回溯路径
            if (now.x == target_x && now.y == target_y) 
            {
                vector<nodes> path;
                int key = now.x * grid_width + now.y;

                while (came_from.find(key) != came_from.end()) // 回溯路径
                {
                    path.push_back(now);//
                    now = all_nodes[came_from[key]];//
                    key = now.x * grid_width + now.y;
                }
                path.push_back(start);
                reverse(path.begin(), path.end());// 路径反序

                //RCLCPP_INFO(this->get_logger(), "找到路了终于");
                //draw_path_on_grid_path(grid, path);

                return path;
            }

            // 获取当前节点的邻居节点
            auto neighbors = get_neighbors(now, grid);
            for (auto& neighbor : neighbors) 
            {
                if (grid[neighbor.x][neighbor.y] == 0)
                {
                    // 计算新的 g 和 h 值
                    neighbor.g = now.g + 1; 
                    neighbor.h = heuristic(neighbor.x, neighbor.y, target_x, target_y);

                    int neighbor_key = neighbor.x * grid_width + neighbor.y;//

                    // 如果该邻居节点尚未被访问过，或者找到了更低的代价路径
                    if (all_nodes.find( ) == all_nodes.end() || neighbor.g < all_nodes[neighbor_key].g) 
                    {
                        all_nodes[neighbor_key] = neighbor; // 更新访问过的节点
                        came_from[neighbor_key] = now.x * grid_width + now.y; // 记录路径
                        open_set.push(neighbor); // 将邻居节点加入优先队列
                    }
                }
            }
        }

        cerr << "没找着" << endl; // 如果没有找到路径，输出错误提示
        return {}; // 返回空路径
    }
    //根据路径，来寻找直角转弯拐点
    vector<nodes> get_path_points(const vector<nodes>& path)
    {
        vector<nodes> key_point;
        if (path.size() < 3) {return path;}
        // 起点总是一个关键点
        key_point.push_back(path[0]);

        for (size_t i = 1; i < path.size() - 1; ++i) {
           
            nodes prev = path[i - 1];
            nodes now = path[i];
            nodes next = path[i + 1];

            // 计算前一个节点和当前节点的方向
            int dir_x_prev = now.x - prev.x;
            int dir_y_prev = now.y - prev.y;

            // 计算当前节点和下一个节点的方向
            int dir_x_next = next.x - now.x;
            int dir_y_next = next.y - now.y;

            // 判断方向是否变化
            if (dir_x_prev != dir_x_next || dir_y_prev != dir_y_next) {
                key_point.push_back(now);
            }
        }

        // 终点总是一个关键点
        key_point.push_back(path.back());

        return key_point;
    }
    //绘制路径，拐点，区域目标点，处理后的地图
    void draw_path_on_grid_path(const vector<vector<int>>& grid, const vector<nodes>& path)
    {
        // 创建白色背景的图像，大小与 grid 相同
        cv::Mat map_image(grid.size(), grid[0].size(), CV_8UC3, cv::Scalar(255, 255, 255)); 

        // 绘制栅格地图
        for (int y = 0; y < grid.size(); ++y) 
        {
            for (int x = 0; x < grid[0].size(); ++x) 
            {
                if (grid[y][x] == 1) // 如果是障碍物
                { 
                    map_image.at<cv::Vec3b>(x, y) = cv::Vec3b(0, 0, 0); // 绘制为黑色
                }
            }
        }

        // 绘制路径
        for (const auto& node : path) 
        {
            int x = node.x; // 获取路径点的 x 坐标
            int y = node.y; // 获取路径点的 y 坐标
            map_image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255); // 将路径绘制为红色
        }

        //绘制key_points
        // for(const auto& key_point : key_points)
        // {
        //     int x = key_point.x; // 获取路径点的 x 坐标
        //     int y = key_point.y; // 获取路径点的 y 坐标
        //     map_image.at<cv::Vec3b>(y, x) = cv::Vec3b(100, 255, 100); // 将路径绘制为红色
        //     //cout<<key_point.x<<" "<<key_point.y<<endl;
        // }
        //map_image.at<cv::Vec3b>(1, 5) = cv::Vec3b(0, 255, 0); // 将路径绘制为红色
        RCLCPP_INFO(this->get_logger(), "Path drawn!");

        for(const auto& center : rough_region_centers)
        {
            //map_image.at<cv::Vec3b>(center.x,center.y) = cv::Vec3b(255, 0, 255);
            //cout<<"rough center (x,y):" << center.x << ", " << center.y << endl;
        }
        //输出详细区域中心点
        for (const auto& center : temp)
        {
            map_image.at<cv::Vec3b>(center.x,center.y) = cv::Vec3b(255, 0, 255);
            cout<<"Detailed center (x,y):" << center.x << ", " << center.y << endl;
        }
        cout << endl;

        // 调整图像大小并显示
        resize(map_image, map_image, cv::Size(1250, 1250));
        cv::imshow("Path_map", map_image);
    }
    //坐标回调函数，用来操控车的运动
    void positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        /***********************处理自身位置信息**************************/
        robot_x_point = msg->pose.position.x;
        robot_y_point = msg->pose.position.y;
    }
    //运动函数
    bool con_move(const nodes start_point, const nodes end_point, const nodes robot)
    {
        //cout<<"开始位置"<<start_point.x<<" "<<start_point.y<<endl;
        //cout<<"实际位置"<<robot.x<<" "<<robot.y<<endl;
        float target_x = end_point.y - start_point.x;float robot_x = end_point.y - robot.x;
        float target_y = end_point.x - start_point.y; float robot_y = end_point.x - robot.y;
        dis_robot_to_target = sqrt(robot_x * robot_x + robot_y * robot_y);//实际位置到目标点的距离(剩余距离)
        float dis_start_to_target = sqrt(target_x * target_x + target_y * target_y);//起始位置到目标点的距离（全程距离）
        float dis_start_to_target_x = abs(target_x); float dis_start_to_target_y = abs(target_y);
        float dis_robot_to_target_x = abs(robot_x);  float dis_robot_to_target_y = abs(robot_y);
        auto target_speed_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
        float dis_shache = 0;
        

        linear_speed_y = robot_y / dis_robot_to_target * 1.6;
        linear_speed_x = robot_x / dis_robot_to_target * 1.6;
        target_speed_msg->twist.linear.y = linear_speed_y;
        target_speed_msg->twist.linear.x = linear_speed_x;
        speed_pub_->publish(*target_speed_msg);//发布速度

        //if(qaz==0)
        {
            if(dis_robot_to_target<=1)
            { 
                cout <<" ******1*************成功到达******************** "<< endl;
                cout<<"--当前位置--:"<<robot.x<<","<<robot.y<<endl;
                cout << endl;
                return true;
            }
            else
            {
                cout<<"--当前位置--:"<<robot.x<<","<<robot.y<<"--目标位置--:"<<end_point.y<<","<<end_point.x<<"--起始位置--"<<start_point.x<<","<<start_point.y<<endl;
                cout<<"还未到达--当前到目标点的距离:"<<dis_robot_to_target<<endl;
                cout << endl;
                return false;
            }
        // }else 
        // {
        //     if(dis_robot_to_target<=3.5)
        //     { 
        //         cout <<" *3*************成功到达************** "<< endl;
        //         cout<<"--当前位置--:"<<robot.x<<","<<robot.y<<endl;
        //         cout << endl;
        //         qaz = 0;
        //         return true;
        //     }
        //     else
        //     {
        //         cout<<"--当前位置--:"<<robot.x<<","<<robot.y<<"--目标位置--:"<<end_point.y<<","<<end_point.x<<"--起始位置--"<<start_point.x<<","<<start_point.y<<endl;
        //         cout<<"还未到达--当前到目标点的距离:"<<dis_robot_to_target<<endl;
        //         cout << endl;
        //         qaz = 0;
        //         return false;
        //     }
        }
    }
    //发送目标点
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        /***********************处理目标位置信息**************************/
        goal_x = msg->pose.position.x;
        goal_y = msg->pose.position.y;
        //RCLCPP_INFO(this->get_logger(), "Goal position received: [x: %f, y: %f, z: %f]", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr real_speed_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr game_start_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Publisher<tdt_interface::msg::SendData>::SharedPtr send_data_pub_;
    rclcpp::Subscription<tdt_interface::msg::ReceiveData>::SharedPtr receive_data_sub_;
    float yaw = 0;float pitch = 0; vector<Point2f>target_center;
    int flag = 0;
    int index = 0;
    float mark_x = 0.0;float mark_y = 0.0;
    float goal_x = 0.0;float goal_y = 0.0;
    int my_flag=0;
    int qaz = 0;
    bool shoot = true;
    vector<Point> rough_region_centers;
    vector<Point> detailed_region_centers;
    vector<Point> temp;
    vector<vector<nodes> >key_points;
    float robot_x_point = 0;
    float robot_y_point = 0;
    float real_linear_speed_x = 0;
    float real_linear_speed_y = 0;
    bool is_shooting = false;
    bool is_navigating = true;
    int light_mark=0;
    float linear_speed_x = 0; float linear_speed_y = 0;
    vector<vector<int>> grid;
    vector<Point> center_points_path;
    vector<vector<nodes>> paths;
    float dis_robot_to_target = 0;
    KalmanFilter_mine kf;
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoShoot>());
    rclcpp::shutdown();
    return 0;
}
