#ifndef PCLMIDROWDETECTION_HPP
#define PCLMIDROWDETECTION_HPP

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_stick.h>

#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <Eigen/Dense>

#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>

#include "string"
#include "reconfigure_handler.hpp"
#include <vineyard_midrow_detection/MidrowDetectionConfig.h>

using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using reconfigureCfg = vineyard_midrow_detection::MidrowDetectionConfig;

struct Bounds
{
    Bounds(): lower_(0.0), upper_(0.0) {};
    Bounds(double lower_bound, double upper_bound)
        :lower_(lower_bound), upper_(upper_bound) {};

    double upper_, lower_;
};

struct Box3d
{
    Box3d(){};
    Box3d(Bounds x_bounds, Bounds y_bounds, Bounds z_bounds)
        : x_bounds_(x_bounds), y_bounds_(y_bounds), z_bounds_(z_bounds) {};

    Bounds x_bounds_, y_bounds_, z_bounds_;
};

struct Line2d
{
    Eigen::Vector2d point_;
    Eigen::Vector2d direction_;
    double angle_deg_;

    Line2d();
    Line2d( Eigen::Vector2d point_in, Eigen::Vector2d direction_in);
    void calcAngleFromDirection();
    double getPointY(double x) const;
    void publish(const ros::Publisher &marker_pub, const std::string &frame_id, double marker_line_length = 6.0) const;
};

struct RowBorders
{
    Line2d right_line_, left_line_;
    bool empty_;
    RowBorders();
    RowBorders(Line2d right_line, Line2d left_line);
    bool isEmpty() const { return empty_; };
    Line2d getMidLine() const;
};

class PclMidRowDetection
{
public:
    PclMidRowDetection(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
private:
    PointCloudXYZ::Ptr input_cloud_, flat_cloud_, filtered_cloud_;
    Box3d keep_box_, remove_box_;

    double line_distance_threshold_, max_line_angle_deg_;
    uint32_t line_detection_min_points_n_;

    std::string lidar_frame_id_;
    
    ros::NodeHandle nh_, nh_private_;
    double rate_;
    ros::Publisher  filtered_cloud_pub_, flat_cloud_pub_, marker_pub_mid_, 
                    marker_pub_left_, marker_pub_right_,
                    n_detected_lines_pub_;
    ros::Subscriber input_pointcloud_sub_;

    std::string input_cloud_topic_;

    void inputCloudCallback(const sensor_msgs::PointCloud2ConstPtr &ros_msg);
    
    void initialize(); 
    void initializeKeepBox();
    void initializeRemoveBox();
                    
    ros::Timer control_timer_;
    void loop(const ros::TimerEvent &);

    void cropBox( PointCloudXYZ::Ptr pointcloud, const Box3d &box, bool set_negative = false ) const;
    void flattenPointcloud( PointCloudXYZ::Ptr pointcloud ) const;

    sensor_msgs::PointCloud2 msgFromPcl(const PointCloudXYZ &input_cloud) const;

    std::vector<Line2d> detected_lines_;
    RowBorders border_lines_;

    std::vector<Line2d> findLines(PointCloudXYZ::Ptr input_cloud) const;
    void extractIndices(PointCloudXYZ::Ptr pointcloud,
                        pcl::PointIndices::Ptr indices) const;

    RowBorders selectBorders(const std::vector<Line2d> &lines) const;

    static void publishInt32(const ros::Publisher &int_pub, int input_int);

    ros_util::ReconfigureHandler<reconfigureCfg> reconfigure_handler_;
    void updateReconfigurableParams();
    
};

#endif /* PCLMIDROWDETECTION_HPP */