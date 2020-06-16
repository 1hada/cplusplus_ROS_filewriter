/*
  author : Jonathan Sanabria
  Write topic information to a file in csv format.
*/
#include <diff_drive_controller/gather_to_file.h>

// for roll, pitch, yaw
#include <tf2/LinearMath/Quaternion.h> 
#include <tf2/LinearMath/Matrix3x3.h>

// to get heading coordinate system
#include <cmath>

namespace gather_to_file
{
  void linearInterpolation( const double num )
  {
    const double ang = 0;
  }

  void GatherToFile::calculate_time_derivatives( std::map< std::string , double > gt_pose )
  {
    // Calculate the time derivatives here in order to rotate
    // the previous coordinates w.r.t. the prev_yaw
    // assures that the current coordinates act like a ground truth.
    // ang is negative to rotate towards 0 yaw
             
    const double ang = -1*gt_pose["pre_yaw"] ;
    x_dot =  ( gt_pose["cur_x"]*cos( ang ) - gt_pose["cur_y"]*sin( ang ))
          -1*( gt_pose["pre_x"]*cos( ang ) - gt_pose["pre_y"]*sin( ang ));
    y_dot =  ( gt_pose["cur_x"]*sin( ang ) + gt_pose["cur_y"]*cos( ang ))
          -1*( gt_pose["pre_x"]*sin( ang ) + gt_pose["pre_y"]*cos( ang ));
    z_dot = gt_pose["cur_z"] - gt_pose["pre_z"] ; // does not depend on yaw rotation // as much

    yaw_dot = gt_pose["cur_yaw"] - gt_pose["pre_yaw"]  ; 
    // do not use for training which moves beyond pi degrees as will probably not work well for the pi -> -pi jumps

    x_dot   = gt_pose["skid_calc_x"] - x_dot   ;
    y_dot   = gt_pose["skid_calc_y"] - y_dot  ;
    z_dot   = 0 - z_dot  ;
    yaw_dot = gt_pose["skid_calc_yaw"] - z_dot  ;


  }


  void GatherToFile::write_to_file( int steerState , 
                                    std::map< std::string , double > *speed ,
                                    std::map< std::string , double > *ang_vel ,
                                    std::map< std::string , double > *imu_ori ,
                                    std::map< std::string , double > *gt_pose ) 
  {

    char buf[1024];

    if( (*gt_pose).count( "pre_yaw" ) > 0 && (*gt_pose).count( "pre_x" ) > 0 
      && (*gt_pose).count( "pre_y" ) > 0 && (*gt_pose).count( "pre_z" ) > 0){
      if (  aug_file.is_open())
      { // csv will represent an augmented matrix 
        calculate_time_derivatives( *gt_pose );

        double left_avg , right_avg ;
        left_avg  = ((*speed)["fl"] + (*speed)["bl"])/2 ;
        right_avg = ((*speed)["fr"] + (*speed)["br"])/2 ;
        left_avg  = ( left_avg == 0 )? std::numeric_limits<double>::min() : left_avg ;
        right_avg = (right_avg == 0 )? std::numeric_limits<double>::min() : right_avg ;

        // the front to back ratio is consistently different 
        // and might later be useful for classification of what steer type to use
        double fb_l_ratio , fb_r_ratio , tmp_denom ;
        // will run through tanh to prevent unbounded ratios
        // dividing by 10 to stretch when it goes to 1 
        tmp_denom = ((*speed)["bl"] == 0 )? std::numeric_limits<double>::min() : (*speed)["bl"] ;
        fb_l_ratio = tanh( ((*speed)["fl"]/tmp_denom )/10 )  ;
        tmp_denom = ((*speed)["br"] == 0 )? std::numeric_limits<double>::min() : (*speed)["br"] ;
        fb_r_ratio = tanh( ((*speed)["fr"]/tmp_denom )/10 ) ;

        double rl_ratio , lr_ratio ;
        rl_ratio = tanh( (right_avg / left_avg)/10  ) ;
        lr_ratio = tanh( (left_avg / right_avg)/10 ) ;
        //                 |                                            |      
        //           CLASS |                 INPUT VARIABLES            | TRUE ODOM DELTA     
        //                 |                                            |      
        //      idx :  0     1    2    3    4    5    6    7    8    9    10                                
        //             ste   sfl  sfr  sbl  sbr  r/l  l/r  fb   fb   io   GT                                
        sprintf( buf ,"%d ,  %f , %f , %f , %f , %f , %f , %f , %f , %d , %f \n"
                      "%d ,  %f , %f , %f , %f , %f , %f , %f , %f , %d , %f \n"
                      "%d ,  %f , %f , %f , %f , %f , %f , %f , %f , %d , %f \n"
                      "%d ,  %f , %f , %f , %f , %f , %f , %f , %f , %d , %f \n",
   steerState , (*speed)["fl"] , (*speed)["fr"] , (*speed)["bl"] , (*speed)["br"] , rl_ratio , lr_ratio , fb_l_ratio , fb_r_ratio , 1    , x_dot, 
   steerState , (*speed)["fl"] , (*speed)["fr"] , (*speed)["bl"] , (*speed)["br"] , rl_ratio , lr_ratio , fb_l_ratio , fb_r_ratio , 1    , y_dot,
   steerState , (*speed)["fl"] , (*speed)["fr"] , (*speed)["bl"] , (*speed)["br"] , rl_ratio , lr_ratio , fb_l_ratio , fb_r_ratio , 1    , z_dot, 
   steerState , (*speed)["fl"] , (*speed)["fr"] , (*speed)["bl"] , (*speed)["br"] , rl_ratio , lr_ratio , fb_l_ratio , fb_r_ratio , 1  , yaw_dot );
        aug_file << buf; 
        ROS_DEBUG_STREAM_THROTTLE(.5, buf ); 

      }
    }
  }



} // namespace four_wheel_steering_controller
