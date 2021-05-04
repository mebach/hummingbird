/**
 * \file pid_controller.h
 * \author Devon Morris <devonmorris1992@gmail.com>
 */

#ifndef HUMMINGBIRD_CONTROLLER_PID_CONTROLLER_
#define HUMMINGBIRD_CONTROLLER_PID_CONTROLLER_

#include <ros/ros.h>
#include <hb_msgs/Command.h>
#include <hb_msgs/State.h>
#include <hb_msgs/ReferenceState.h>
#include <time.h>
#include <std_msgs/Float32.h>

namespace hummingbird_controller
{

/**
 * \brief ROS hummingbird pid controller
 */

class PIDController
{
public:
    PIDController();

private:
    // Params for Phi
    float P_phi_;
    float I_phi_;
    float D_phi_;
    float Int_phi_;
    float prev_phi_;

    // Params for Theta
    float theta_r_;
    float P_theta_;
    float I_theta_;
    float D_theta_;
    float Int_theta_;
    float prev_theta_;

    // Params for Psi
    float psi_r_;
    float P_psi_;
    float I_psi_;
    float D_psi_;
    float Int_psi_;
    float prev_psi_;

    ros::Time prev_time_;

    // Hummingbird parameters
    float l1_  = 0;
    float l2_  = 0;
    float l3x_ = 0;
    float l3y_ = 0;
    float l3z_ = 0;
    float lT_  = 0;
    float d_   = 0;
    float m1_  = 0;
    float m2_  = 0;
    float m3_  = 0;
    float J1x_ = 0;
    float J1y_ = 0;
    float J1z_ = 0;
    float J2x_ = 0;
    float J2y_ = 0;
    float J2z_ = 0;
    float J3x_ = 0;
    float J3y_ = 0;
    float J3z_ = 0;
    float Bphi_= 0;
    float Bth_ = 0;
    float Bpsi_= 0;
    float km_  = 0;
    float g_   = 0;


    float Fl_;
    float Fe_;



    void stateCallback(hb_msgs::State::ConstPtr msg);
    void referenceStateCallback(hb_msgs::ReferenceState::ConstPtr msg);


    ros::Publisher command_pub_;
    ros::Subscriber state_sub_;
    ros::Subscriber reference_state_sub_;
};

} // namespace pid_controller
#endif
