/*
  author : Jonathan Sanabria
  Write topic information to a file in csv format.
*/
#pragma once
#include <iostream>
#include <fstream>

#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <diff_drive_controller/imu_info.h> 
#include <boost/thread/thread.hpp>

namespace gather_to_file
{
  /**
   * \brief File writing class that also gets the ground truth location for later pose estimation based on desired parameters
   */
  class GatherToFile
  {
  public:

    /**
     * \brief Constructor
     * set file name for the eventual matrix operations equal to 
     * \param augmented_file_name  
     */
    GatherToFile( std::string augmented_file_name )
    //: gt_cb_info()
    {

      if (augmented_file_name.empty() ){
        augmented_matrix_file_name = "af.csv";
      } else {
        augmented_matrix_file_name = "/Workspace/work/directory_name/" + augmented_file_name ;
      }
      aug_file.open(augmented_matrix_file_name, std::ofstream::out | std::ofstream::app);
      ROS_ASSERT( aug_file.is_open() );
      file_is_augmented = true ;


    }
    /**
     * \brief Destructor 
     */
    ~GatherToFile(){

      if (  aug_file.is_open()){   aug_file.close(); }
      if (param_file.is_open()){ param_file.close(); }
      if ( pose_file.is_open()){  pose_file.close(); }

    }

    void write_to_file( int steerState , 
                          std::map< std::string , double > *speed ,
                          std::map< std::string , double > *ang_vel ,
                          std::map< std::string , double > *imu_ori ,
                          std::map< std::string , double > *gt_pose );

    void calculate_time_derivatives( std::map< std::string , double > gt_pose  );
  private :

    bool file_is_augmented ;
    std::ofstream   aug_file ;
    std::ofstream param_file ;
    std::ofstream  pose_file ;

    // file names
    std::string augmented_matrix_file_name ;
    std::string     param_matrix_file_name ;
    std::string      pose_matrix_file_name ;

    /// Current time derivative :
    double x_dot;        //   [m]
    double y_dot;        //   [m]
    double z_dot;        //   [m]
    double yaw_dot;      //   [rad]

    ros::NodeHandle n;

  };
}
