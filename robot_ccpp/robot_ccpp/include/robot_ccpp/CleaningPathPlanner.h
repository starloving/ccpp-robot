/***
 * @brief: cleaning robot path planning
 * @author: Benny
 * @date: 2023.10
***/

#ifndef CLEANINGPATHPLANNING_H
#define CLEANINGPATHPLANNING_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>
#include <mutex>


#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"

// #include "tf/tf.h"
// #include "tf/transform_listener.h"
// #include <nav2_costmap_2d/nav2_costmap_2d.h>
// #include <nav2_costmap_2d/nav2_costmap_2d_ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Path.h>

using namespace cv;
using namespace std;

constexpr double PI = 3.14159;

struct cellIndex
{
    int row;
    int col;
    double theta; //{0, 45,90,135,180,225,270,315}     角度信息   hjr 注
};


/*************************************************
 *
 * 读取栅格地图并根据占据信息获取其对应的空闲（可行走）空间，
 * 按照遍历算法规划行走路线。
 *
 * **********************************************/
class CleaningPathPlanning : public rclcpp::Node
{
    public: 
        //CleaningPathPlanning() = delete;
        CleaningPathPlanning();

        vector<geometry_msgs::msg::PoseStamped> GetPathInROS();
        vector<geometry_msgs::msg::PoseStamped> GetBorderTrackingPathInROS();

        void SetCoveredGrid(double wx, double wy);
        int GetSizeOfCell() { return this->SIZE_OF_CELL; }
        bool Boundingjudge(int a, int b);
        //for visualization
        void PublishCoveragePath();
        void PublishGrid();


    private:
        //helper functions.
        bool initializeMats();
        bool initializeCoveredGrid();
        void getCellMatAndFreeSpace(Mat srcImg, Mat &cellMat, vector<cellIndex> &freeSpaceVec);
        void initializeNeuralMat(Mat cellMat, Mat neuralizedMat);
        void writeResult(Mat resultmat, vector<cellIndex> pathVec);
        void writeResult(cv::Mat resultmat, std::vector<cv::Point2i> pathVec);
        void mainPlanningLoop();
        double distance(Point2i pta, Point2i ptb);
        bool findElement(vector<cv::Point2i> pointsVec, cv::Point2i pt, int &index);
        void publishPlan(const std::vector<geometry_msgs::msg::PoseStamped> &path);
        bool cellContainsPoint(cv::Point2i pt, cellIndex cell);

        void GetBorderTrackingPathInCV(vector<cv::Point2i> &resultVec);
        vector<cellIndex> GetPathInCV();

        bool initialized_;
        Mat srcMap_;
        Mat cellMat_;
        Mat neuralizedMat_;
        vector<cellIndex> freeSpaceVec_;
        vector<cellIndex> pathVec_;
        vector<geometry_msgs::msg::PoseStamped> pathVecInROS_;

        double resolution_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_ ;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_ ;

        nav_msgs::msg::OccupancyGrid covered_path_grid_;

        //tf::TransformListener &tf_;
        geometry_msgs::msg::PoseStamped initPose_;
        
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap2d_ros_;
        std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
        nav2_costmap_2d::Costmap2D * costmap2d_;
        
        int SIZE_OF_CELL; //must be odd number.
        int GRID_COVERED_VALUE;
};

#endif // CLEANINGPATHPLANNING_H
