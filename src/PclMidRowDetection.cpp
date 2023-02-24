#include "PclMidRowDetection.hpp"
#include "ChronoCall.hpp"

PclMidRowDetection::PclMidRowDetection(ros::NodeHandle &nh, ros::NodeHandle &nh_private) 
  : input_cloud_(boost::make_shared<PointCloudXYZ>()), 
    filtered_cloud_(boost::make_shared<PointCloudXYZ>()),
    flat_cloud_(boost::make_shared<PointCloudXYZ>()), 
    rate_(50), nh_(nh), nh_private_(nh_private), reconfigure_handler_("vineyard_midrow_detection") 
{
    initialize();
}

void PclMidRowDetection::initialize()
{
  nh_private_.param<double>("rate", rate_, 50);
  nh_private_.param<std::string>("input_pointcloud_topic", input_cloud_topic_, "/rslidar_points");

  filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1);
  flat_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("flat_cloud", 1);
  
  marker_pub_mid_ = nh_.advertise<visualization_msgs::Marker>("middle_line", 1);
  marker_pub_left_ = nh_.advertise<visualization_msgs::Marker>("left_line", 1);
  marker_pub_right_ = nh_.advertise<visualization_msgs::Marker>("right_line", 1);
  marker_pub_next_left_ = nh_.advertise<visualization_msgs::Marker>("next_left_line", 1);
  marker_pub_next_right_ = nh_.advertise<visualization_msgs::Marker>("next_right_line", 1);

  pure_pursuit_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("pure_pursuit_point", 1);

  input_pointcloud_sub_ = nh_.subscribe( input_cloud_topic_, 1, &PclMidRowDetection::inputCloudCallback, this);

  control_timer_ = nh_.createTimer(ros::Duration(ros::Rate(rate_)), &PclMidRowDetection::loop, this);

  n_detected_lines_pub_ = nh_.advertise<std_msgs::Int32>("number_of_detected_lines", 1);
  line_distance_threshold_ = 0.4;
  pure_pursuit_point_distance_ = 1.5;
}

void PclMidRowDetection::loop(const ros::TimerEvent &/* unused */)
{
  updateReconfigurableParams();

  pcl::copyPointCloud(*input_cloud_, *filtered_cloud_);

  cropBox(filtered_cloud_, remove_box_, true); //Delete points in remove_box_
  cropBox(filtered_cloud_, keep_box_, false); //Delete points outside of keep_box_

  pcl::copyPointCloud(*filtered_cloud_, *flat_cloud_);

  flattenPointcloud(flat_cloud_);
  if(flat_cloud_->points.empty()) return;
  
  filtered_cloud_pub_.publish( msgFromPcl(*filtered_cloud_) );
  flat_cloud_pub_.publish( msgFromPcl(*flat_cloud_) );
  
  detected_lines_ = findLines(flat_cloud_);
  publishInt32(n_detected_lines_pub_, detected_lines_.size());
  
  border_lines_ = selectBorders(detected_lines_);
  next_border_lines_ = selectNextRowBorders(detected_lines_, border_lines_);
  
  if (border_lines_.isEmpty()) return; //TODO: publish something
  
  Line2d mid_line = border_lines_.getMidLine();

  border_lines_.right_line_.publish(marker_pub_right_, lidar_frame_id_);
  border_lines_.left_line_.publish(marker_pub_left_, lidar_frame_id_);
  if( next_border_lines_.right_line_.angle_deg_ != border_lines_.right_line_.angle_deg_ && 
      next_border_lines_.right_line_.point_ != border_lines_.right_line_.point_ )
  {
    next_border_lines_.right_line_.publish(marker_pub_next_right_, lidar_frame_id_);
  }
  if( next_border_lines_.left_line_.angle_deg_ != border_lines_.left_line_.angle_deg_ && 
      next_border_lines_.left_line_.point_ != border_lines_.left_line_.point_ )
  {
    next_border_lines_.left_line_.publish(marker_pub_next_left_, lidar_frame_id_);
  }
  
  mid_line.publish(marker_pub_mid_, lidar_frame_id_);
  publishPurePursuitPoint(mid_line);
}

void PclMidRowDetection::inputCloudCallback (const sensor_msgs::PointCloud2ConstPtr &ros_msg)
{
  pcl::fromROSMsg(*ros_msg, *input_cloud_);
  lidar_frame_id_ = ros_msg->header.frame_id;
}

void PclMidRowDetection::updateReconfigurableParams()
{
  auto reconfigure_data = reconfigure_handler_.getData();
  line_distance_threshold_ = reconfigure_data.line_detection_distance_threshold;
  max_line_angle_deg_ = reconfigure_data.line_detection_max_angle;
  line_detection_min_points_n_ = reconfigure_data.line_detection_min_n_points;


  remove_box_.x_bounds_ = Bounds( reconfigure_data.remove_box_lower_bound_x, 
                                  reconfigure_data.remove_box_upper_bound_x );
  remove_box_.y_bounds_ = Bounds( reconfigure_data.remove_box_lower_bound_y, 
                                  reconfigure_data.remove_box_upper_bound_y );
  remove_box_.z_bounds_ = Bounds( reconfigure_data.remove_box_lower_bound_z, 
                                  reconfigure_data.remove_box_upper_bound_z );

  keep_box_.x_bounds_ = Bounds(   reconfigure_data.keep_box_lower_bound_x, 
                                  reconfigure_data.keep_box_upper_bound_x );
  keep_box_.y_bounds_ = Bounds(   reconfigure_data.keep_box_lower_bound_y, 
                                  reconfigure_data.keep_box_upper_bound_y );
  keep_box_.z_bounds_ = Bounds(   reconfigure_data.keep_box_lower_bound_z, 
                                  reconfigure_data.keep_box_upper_bound_z );
}

void PclMidRowDetection::publishInt32(const ros::Publisher &int_pub, int input_int)
{
  std_msgs::Int32 int_msg;
  int_msg.data = input_int;
  int_pub.publish(int_msg);
}

void PclMidRowDetection::cropBox(PointCloudXYZ::Ptr pointcloud, const Box3d &box, bool set_negative) const
{
  pcl::CropBox<pcl::PointXYZ> boxFilter;
  if(set_negative) boxFilter.setNegative(true);
  
  boxFilter.setMin(Eigen::Vector4f(box.x_bounds_.lower_, box.y_bounds_.lower_, box.z_bounds_.lower_, 1.0));
  boxFilter.setMax(Eigen::Vector4f(box.x_bounds_.upper_, box.y_bounds_.upper_, box.z_bounds_.upper_, 1.0));
  boxFilter.setInputCloud(pointcloud);
  boxFilter.filter(*pointcloud);
}

void PclMidRowDetection::flattenPointcloud(PointCloudXYZ::Ptr pointcloud) const
{
  for(auto &point : pointcloud->points) point.z = 0.0;
}

sensor_msgs::PointCloud2 PclMidRowDetection::msgFromPcl(const PointCloudXYZ &input_cloud) const
{
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(input_cloud, msg);
  return msg;
}

std::vector<Line2d> PclMidRowDetection::findLines(PointCloudXYZ::Ptr input_cloud)  const
{
  std::vector<Line2d> temp_lines;
  auto temp_cloud = boost::make_shared<PointCloudXYZ>();
  
  pcl::copyPointCloud(*input_cloud, *temp_cloud);
  
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  auto inliers = boost::make_shared<pcl::PointIndices>();

  auto coefficients = boost::make_shared<pcl::ModelCoefficients>();
  auto cloud_plane = boost::make_shared<PointCloudXYZ>();
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_LINE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  while(temp_cloud->points.size() > line_detection_min_points_n_)
  {
    seg.setInputCloud (temp_cloud);
    seg.setDistanceThreshold (line_distance_threshold_);
    seg.segment (*inliers, *coefficients);

    Line2d temp_line( Eigen::Vector2d(coefficients->values[0], coefficients->values[1]),
                      Eigen::Vector2d(coefficients->values[3], coefficients->values[4]) ); 

    temp_lines.push_back(temp_line);

    extractIndices(temp_cloud, inliers);
  }

  return temp_lines;
}


RowBorders PclMidRowDetection::selectBorders(const std::vector<Line2d> &lines) const
{
  int right_line_index = -1;
  int left_line_index = -1;
  double inf = 9999;
  double min_positive_y = inf;
  double max_negative_y = -inf;

  for(uint32_t i = 0; i < lines.size(); i++)
  {
    double current_y = lines[i].getPointY(0.0);
    
    if(abs(lines[i].angle_deg_) > max_line_angle_deg_) //Disregard lines with large angles
      continue;

    //Is the current line the closest line to the left?        
    if (current_y < 0 && current_y > max_negative_y)
    {
      max_negative_y = current_y;
      left_line_index = i;
    } 
    
    //Is the current line the closest line to the right?
    if (current_y > 0 && current_y < min_positive_y) 
    {
      min_positive_y = current_y;
      right_line_index = i;
    } 
  }

  if (right_line_index == -1 || left_line_index == -1) //right or left line don't exist
    return RowBorders();
  else 
    return RowBorders(lines[right_line_index], lines[left_line_index]);
}

RowBorders PclMidRowDetection::selectNextRowBorders(const std::vector<Line2d> &lines, const RowBorders &row_borders) const
{
  int right_line_index = -1;
  int left_line_index = -1;
  double inf = 9999;
  double min_positive_y = inf;
  double max_negative_y = -inf;

  double next_line_threshold = 0.1;

  for(uint32_t i = 0; i < lines.size(); i++)
  {
    double current_y = lines[i].getPointY(0.0);
    
    if(abs(lines[i].angle_deg_) > max_line_angle_deg_) //Disregard lines with large angles
      continue;

    //Is the current line the closest line to the left?        
    if (current_y < row_borders.left_line_.getPointY(0.0) - next_line_threshold && current_y > max_negative_y)
    {
      max_negative_y = current_y;
      left_line_index = i;
    } 
    
    //Is the current line the closest line to the right?
    if (current_y > row_borders.right_line_.getPointY(0.0) + next_line_threshold && current_y < min_positive_y) 
    {
      min_positive_y = current_y;
      right_line_index = i;
    } 
  }

  if (right_line_index == -1) //right line doesn't exist
    return RowBorders(row_borders.right_line_, lines[left_line_index]);
  if (left_line_index == -1) //right line doesn't exist
    return RowBorders(lines[right_line_index], row_borders.left_line_);
  
  return RowBorders(lines[right_line_index], lines[left_line_index]);
}


void PclMidRowDetection::extractIndices(PointCloudXYZ::Ptr pointcloud,
                                        pcl::PointIndices::Ptr indices) const
{
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(pointcloud);
  extract.setIndices(indices);
  extract.setNegative(true);
  extract.filter(*pointcloud);
}

void PclMidRowDetection::publishPurePursuitPoint(Line2d mid_line) const
{
  geometry_msgs::PointStamped msg;
  msg.header.frame_id = lidar_frame_id_;
  msg.header.stamp = ros::Time::now();

  
  msg.point.x = pure_pursuit_point_distance_;
  msg.point.y = mid_line.getPointY(pure_pursuit_point_distance_);
  msg.point.z = 0.0;

  pure_pursuit_point_pub_.publish(msg);
}

//--------------------------------Line2d--------------------------------
Line2d::Line2d() {};
Line2d::Line2d( Eigen::Vector2d point_in, Eigen::Vector2d direction_in) 
  : point_(std::move(point_in)), direction_(std::move(direction_in))  
{
  calcAngleFromDirection();
}

double Line2d::getPointY(double x) const
{
  double t;
  t = (x - point_(0)) / direction_(0);
  double y = point_(1) + t * direction_(1);
  return y;
}

void Line2d::calcAngleFromDirection()
{
  angle_deg_ = atan2(direction_(1), direction_(0)) * 180.0 / 3.14159;
}


void Line2d::publish(const ros::Publisher &marker_pub, const std::string &frame_id, double marker_line_length) const
{
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = frame_id;
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "points_and_lines";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  line_strip.scale.x = 0.1;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  Eigen::Vector2d start_point_eigen(-marker_line_length/2, getPointY(-marker_line_length/2));
  geometry_msgs::Point start_point;
  start_point.x = start_point_eigen(0);
  start_point.y = start_point_eigen(1);
  start_point.z = 0.0;

  Eigen::Vector2d end_point_eigen(marker_line_length/2, getPointY(marker_line_length/2));
  geometry_msgs::Point end_point;
  end_point.x = end_point_eigen(0);
  end_point.y = end_point_eigen(1);
  end_point.z = 0.0;

  line_strip.points.push_back(start_point);
  line_strip.points.push_back(end_point);

  marker_pub.publish(line_strip);
}



//--------------------------------RowBorders--------------------------------

RowBorders::RowBorders() : empty_(true)
{}
RowBorders::RowBorders(Line2d right_line, Line2d left_line) : empty_(false), left_line_(left_line), right_line_(right_line)
{}

Line2d RowBorders::getMidLine() const
{
  Line2d mid_line(Eigen::Vector2d((right_line_.point_(0) + left_line_.point_(0)) / 2.0,
                                  (right_line_.point_(1) + left_line_.point_(1)) / 2.0),
                  Eigen::Vector2d((right_line_.direction_(0) + left_line_.direction_(0)) / 2.0,
                                  (right_line_.direction_(1) + left_line_.direction_(1)) / 2.0));
  return mid_line;
}
