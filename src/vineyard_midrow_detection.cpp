#include "PclMidRowDetection.hpp"

int main(int argc, char** argv) 
{
  ros::init (argc, argv, "vineyard_midrow_detection");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  PclMidRowDetection mid_row_detection(nh, nh_private);
    
  ros::spin();
  
  return (0);
}
