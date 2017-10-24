    #include "ros/ros.h" 
    #include "std_msgs/Bool.h" 
    #include "std_msgs/String.h" 
    #include "std_msgs/Float64.h" 
    #include "geometry_msgs/Twist.h" 
    #include <unistd.h>

    #include "sensor_msgs/Joy.h"
    #include <iostream> 

  	//#include "mybot_msg/msgMybot_jointInterface.h"
    #include "mybot_msg/msgMybot_basicMovement.h"
    #include "mybot_msg/msgMybot_detailMovement.h"
 

  double rosTimeToDouble(ros::Time RosTime) //ToDo implement in a library
  { 
    double a;
    double b;
    double c;
    a=RosTime.sec;
    b=RosTime.nsec;
    c=1.0*a + 1.0*b*10.0e-10;
    return c;
  } 

  class BasicMovementControl
  {

  public:

	  BasicMovementControl(){
   		ROS_INFO("in emty constructor");


   	}

	  ~BasicMovementControl(){}

	  void Reset(){}

	  void Load(ros::NodeHandle n){

		ROS_INFO("in Load");
		//init 
        x_ = 0.0; 
        y_ = 0.0; 
        zrot_ = 0.0;

        left_front_leg_ = 0;
      	left_back_leg_ = 0;
      	right_front_leg_ = 0;
      	right_back_leg_ = 0;

      	mec_left_front_ =  0;
        mec_left_back_ = 0;
        mec_right_front_ =   0;
        mec_right_back_ =  0;


        ChassisVelPublisher_ = n.advertise<geometry_msgs::Twist>("mybot/base_link_diffc/cmd_vel", 10);
        BaseWheelLeftPublisher_ = n.advertise<std_msgs::Float64>("/mybot/base_left_vc/command", 10);
        BaseWheelRichtPublisher_ = n.advertise<std_msgs::Float64>("/mybot/base_right_vc/command", 10);

        LeftFrontLegAnglePublisher_ = n.advertise<std_msgs::Float64>("/mybot/leg_left_front_pc/command", 10);
        LeftBackLegAnglePublisher_ = n.advertise<std_msgs::Float64>("/mybot/leg_left_back_pc/command", 10);
        RightFrontLegAnglePublisher_ = n.advertise<std_msgs::Float64>("/mybot/leg_right_front_pc/command", 10);
        RightBackLegAnglePublisher_ = n.advertise<std_msgs::Float64>("/mybot/leg_right_back_pc/command", 10);

        LeftFrontLegMecPublisher_ = n.advertise<std_msgs::Float64>("/mybot/mw_left_front_vc/command", 10);
        LeftBackLegMecPublisher_ = n.advertise<std_msgs::Float64>("/mybot/mw_left_back_vc/command", 10);
        RightFrontLegMecPublisher_ = n.advertise<std_msgs::Float64>("/mybot/mw_right_front_vc/command", 10);
        RightBackLegMecPublisher_ = n.advertise<std_msgs::Float64>("/mybot/mw_right_back_vc/command", 10);

        DetailMovementPublisher_ = n.advertise<mybot_msg::msgMybot_detailMovement>("mybot/detail/cmdMovement", 10);
		RobotBasicMovementubscriber_ =  n.subscribe("mybot/robot/cmdBasicMovement",10, &BasicMovementControl::RobotcmdBasicMovementCallback ,this); 



	  }

    void loop()
    {
        ROS_INFO("in loop");
        transChassisWheelSpeed();
        transLegAngle();
        wheelMovement();
        CmdDetailMovementPublish();
        
    }



  private:
  	//Modes:
  	bool leg_velocity_mode_;
    bool mecanum_movement_mode_;

    double x_; 
    double y_; 
    double zrot_;
    geometry_msgs::Twist chassisMovCmd_;

    double bw_left_;
    double bw_right_;

    double cmd_left_front_leg_;
    double cmd_left_back_leg_;
    double cmd_right_front_leg_;
    double cmd_right_back_leg_;
    double left_front_leg_;
    double left_back_leg_;
    double right_front_leg_;
    double right_back_leg_;

    double mec_left_front_;
    double mec_left_back_;
    double mec_right_front_;
    double mec_right_back_;

    std_msgs::Float64 fbw_left_;
    std_msgs::Float64 fbw_right_;

    std_msgs::Float64 fleft_front_leg_;
    std_msgs::Float64 fleft_back_leg_;
    std_msgs::Float64 fright_front_leg_;
    std_msgs::Float64 fright_back_leg_;

    std_msgs::Float64 fmec_left_front_;
    std_msgs::Float64 fmec_left_back_;
    std_msgs::Float64 fmec_right_front_;
    std_msgs::Float64 fmec_right_back_;

    ros::Publisher ChassisVelPublisher_;
    ros::Publisher BaseWheelLeftPublisher_;
    ros::Publisher BaseWheelRichtPublisher_;

    ros::Publisher LeftFrontLegAnglePublisher_;
    ros::Publisher LeftBackLegAnglePublisher_;
    ros::Publisher RightFrontLegAnglePublisher_;
    ros::Publisher RightBackLegAnglePublisher_;

    ros::Publisher LeftFrontLegMecPublisher_;
    ros::Publisher LeftBackLegMecPublisher_;
    ros::Publisher RightFrontLegMecPublisher_;
    ros::Publisher RightBackLegMecPublisher_;


    ros::Publisher DetailMovementPublisher_;
    ros::Subscriber RobotBasicMovementubscriber_;

    
    //ToDo!!!
    double wheel_separation_; //ToDo in parameter?!!!

    mybot_msg::msgMybot_detailMovement cmdDetailMovement_; 
    
    

    //Subscriber / Callback Functions
    void RobotcmdBasicMovementCallback(const mybot_msg::msgMybot_basicMovement::ConstPtr& cmd_msg)
    {
      x_ = cmd_msg->robot_x ;
      y_ = cmd_msg->robot_y ;
      zrot_ = cmd_msg->robot_zrot ;


      cmd_left_front_leg_ = cmd_msg->left_front_leg;
      cmd_left_back_leg_ = cmd_msg->left_back_leg;
      cmd_right_front_leg_ = cmd_msg->right_front_leg;
      cmd_right_back_leg_ = cmd_msg->right_back_leg;

      leg_velocity_mode_ = cmd_msg->leg_velocity_mode;
      mecanum_movement_mode_ = cmd_msg->mecanum_movement_mode;

    }


    //Process Functions
    void transChassisWheelSpeed()
    {
      double R  = 0.1; //radius of wheels
      double vr = x_;
      double va = zrot_;
      wheel_separation_ = 0.3; //ToDo in parameter!!!

      if((mecanum_movement_mode_ == false) ){

        cmdDetailMovement_.left_wheel = (vr + va * wheel_separation_ / 2.0)/R ;
        cmdDetailMovement_.right_wheel = (vr - va * wheel_separation_ / 2.0)/R;
      }

      chassisMovCmd_.linear.x = x_;
      chassisMovCmd_.angular.z = zrot_;

      ChassisVelPublisher_.publish(chassisMovCmd_);
      //ROS_INFO("BasicMovementController: obj: %i cmdWheelVelocity_.left: %f,cmdWheelVelocity_.right: %f, ", this, cmdWheelVelocity_.left,cmdWheelVelocity_.right);
      ROS_INFO("BasicMovementController: cmdDetailMovement_.left_wheel: %f,cmdDetailMovement_.right_wheel: %f, ", cmdDetailMovement_.left_wheel,cmdDetailMovement_.right_wheel);
    }


        //Publisher Functions
    void transLegAngle()
    {
      
      

      if((leg_velocity_mode_ == true) || (leg_velocity_mode_ == false) ){

        // do something here for differentiation of the modes.
      	left_front_leg_ = cmd_left_front_leg_;
      	left_back_leg_ = cmd_left_back_leg_;
      	right_front_leg_ = cmd_right_front_leg_;
      	right_back_leg_ = cmd_right_back_leg_;
      }


      cmdDetailMovement_.base_to_left_front_leg = left_front_leg_;
      cmdDetailMovement_.base_to_left_back_leg = left_back_leg_;
      cmdDetailMovement_.base_to_right_front_leg = left_front_leg_;
      cmdDetailMovement_.base_to_right_back_leg = left_back_leg_;

      fleft_front_leg_.data = left_front_leg_;
      fleft_back_leg_.data = left_back_leg_;
      fright_front_leg_.data = right_front_leg_;
      fright_back_leg_.data = right_back_leg_;


      LeftFrontLegAnglePublisher_.publish(fleft_front_leg_);
      LeftBackLegAnglePublisher_.publish(fleft_back_leg_);
      RightFrontLegAnglePublisher_.publish(fright_front_leg_);
      RightBackLegAnglePublisher_.publish(fright_back_leg_);
    }

    void wheelMovement()
    {

      double fac = 1;

      if((mecanum_movement_mode_ == false) ){

        y_ = 0;
      }

      chassisMovCmd_.linear.x = x_;
      chassisMovCmd_.angular.z = zrot_;

      bw_left_ = fac * x_  + fac * zrot_;
      bw_right_ = fac * x_ -  fac * zrot_;
      mec_left_front_ =  fac * x_ -  fac * y_ + fac * zrot_;
      mec_left_back_ = fac * x_ +  fac * y_ +  fac * zrot_;
      mec_right_front_ =   fac * x_ +  fac * y_ -  fac * zrot_;
      mec_right_back_ =  fac * x_ -  fac * y_ -  fac * zrot_;


      cmdDetailMovement_.left_wheel = bw_left_;
      cmdDetailMovement_.right_wheel = bw_left_;
      cmdDetailMovement_.wheel_to_left_front_leg = mec_left_front_;
      cmdDetailMovement_.wheel_to_left_back_leg = mec_left_back_;
      cmdDetailMovement_.wheel_to_right_front_leg = mec_right_front_;
      cmdDetailMovement_.wheel_to_right_back_leg = mec_right_back_;

      fbw_left_.data = bw_left_;
      fbw_right_.data = bw_right_;
      fmec_left_front_.data = mec_left_front_;
      fmec_left_back_.data = mec_left_back_;
      fmec_right_front_.data = mec_right_front_;
      fmec_right_back_.data = mec_right_back_;

      ChassisVelPublisher_.publish(chassisMovCmd_);
      BaseWheelLeftPublisher_.publish(fbw_left_);
      BaseWheelRichtPublisher_.publish(fbw_right_);

      LeftFrontLegMecPublisher_.publish(fmec_left_front_);
      LeftBackLegMecPublisher_.publish(fmec_left_back_);
      RightFrontLegMecPublisher_.publish(fmec_right_front_);
      RightBackLegMecPublisher_.publish(fmec_right_back_);
    }


    void CmdDetailMovementPublish()
    {
      ros::Time current_time = ros::Time::now();
      cmdDetailMovement_.header.stamp = current_time;
      cmdDetailMovement_.header.frame_id = "command wheel velocity";

      DetailMovementPublisher_.publish(cmdDetailMovement_);
    }


  };


     
 
    int main(int argc, char** argv) 
    {
        ros::init(argc, argv,"basicMovementControl"); 
        double actualTime;
        double lastLoopTime;

        ROS_INFO("in main");
    
        ros::NodeHandle n;
      
        BasicMovementControl BasicMovementController;
        BasicMovementController.Load(n);

        //ros::spin(); 

      
        while (ros::ok())
        {
        actualTime = rosTimeToDouble( ros::Time::now());

        if((actualTime-lastLoopTime) >= 0.1 ){
          BasicMovementController.loop();
          lastLoopTime = actualTime;
        }
          
          ros::spinOnce();
          //usleep(100);
          //sleep(1);

        }

        ROS_INFO("in main end");

    } 