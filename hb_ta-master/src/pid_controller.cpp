/**
 * \file pid_controller.cpp
 * \author Devon Morris <devonmorris1992@gmail.com>
 * \maintainer Mark Petersen <markiepetersen@gmail.com>
 */

#include "pid_controller.h"
#include <math.h>
#include <iostream>

namespace hummingbird_controller
{

PIDController::PIDController()
{
    ros::NodeHandle nh;
    state_sub_ = nh.subscribe("/hb_state", 5, &PIDController::stateCallback, this);
    reference_state_sub_ = nh.subscribe("/hb_reference_state", 5, &PIDController::referenceStateCallback, this);

    command_pub_ = nh.advertise<hb_msgs::Command>("/hb_command", 5);

    nh.getParam("/hummingbird/l1", l1_);
    nh.getParam("/hummingbird/l2", l2_);
    nh.getParam("/hummingbird/l3x", l3x_);
    nh.getParam("/hummingbird/l3y", l3y_);
    nh.getParam("/hummingbird/l3z", l3z_);
    nh.getParam("/hummingbird/lT", lT_);
    nh.getParam("/hummingbird/d", d_);
    nh.getParam("/hummingbird/m1", m1_);
    nh.getParam("/hummingbird/m2", m2_);
    nh.getParam("/hummingbird/m3", m3_);
    nh.getParam("/hummingbird/J1x", J1x_);
    nh.getParam("/hummingbird/J1y", J1y_);
    nh.getParam("/hummingbird/J1z", J1z_);
    nh.getParam("/hummingbird/J2x", J2x_);
    nh.getParam("/hummingbird/J2y", J2y_);
    nh.getParam("/hummingbird/J2z", J2z_);
    nh.getParam("/hummingbird/J3x", J3x_);
    nh.getParam("/hummingbird/J3y", J3y_);
    nh.getParam("/hummingbird/J3z", J3z_);
    nh.getParam("/hummingbird/Bphi", Bphi_);
    nh.getParam("/hummingbird/Bth", Bth_);
    nh.getParam("/hummingbird/Bpsi", Bpsi_);
    nh.getParam("/hummingbird/km", km_);
    nh.getParam("/hummingbird/g", g_);


    Fe_ = (m1_*l1_ + m2_*l2_)*g_/lT_;
    // std::cout << "FE: " << Fe_ << std::endl;
    float JT = m1_*pow(l1_,2)+m2_*pow(l2_,2) + J2z_ + m3_*(pow(l3x_,2)+pow(l3y_,2));
    float b_psi = (lT_*Fe_)/(J1z_+JT);
    float b_phi = 1/J1x_;
    float b_theta = lT_/(m1_*pow(l1_,2)+m2_*pow(l2_,2)+J1y_+J2y_);
    float tr_psi = 1;
    float tr_phi = .25;
    float tr_theta = .5;

    P_phi_ = pow(2.2/tr_phi,2)/b_phi;
    I_phi_ = 0.00;
    D_phi_ = 2*(2.2/tr_phi)*.8/b_phi;

    P_theta_ = pow(2.2/tr_theta,2)/b_theta;
    I_theta_ = 0.05;
    D_theta_ = 2*(2.2/tr_theta)*.8/b_theta;

    P_psi_ = pow(2.2/tr_psi, 2)/b_psi;
    I_psi_ = 0.02;
    D_psi_ = 2*(2.2/tr_psi)*.8/b_psi;

    theta_r_ = 0;
    psi_r_ = 0;

    prev_time_ = ros::Time::now();

}

void PIDController::stateCallback(hb_msgs::State::ConstPtr msg)
{

    float phi = msg->roll;
    float theta = msg->pitch;
    float psi = msg->yaw;


    Fl_ = (m1_*l1_ + m2_*l2_)*g_/lT_*cos(theta);

    ros::Time now = ros::Time::now();
    float dt = (now - prev_time_).toSec();
    prev_time_ = now;

    // Calculate Yaw Control
    float dpsi = (psi - prev_psi_)/dt;
    prev_psi_ = psi;
    float epsi = psi_r_-psi;
    Int_psi_ += epsi*dt;
    // This part comes from successive loop closure
    float phi_r_ = P_psi_*epsi - D_psi_*dpsi + I_psi_*Int_psi_;

    // Calculate Roll Control
    float dphi = (phi - prev_phi_)/dt;
    prev_phi_ = phi;
    float ephi = phi_r_ - phi;
    Int_phi_ += ephi*dt;
    // Calculate torque
    float T_out = P_phi_*ephi - D_phi_*dphi + I_phi_*Int_phi_;

    // Calculate Pitch Control
    float dtheta = (theta - prev_theta_)/dt;
    prev_theta_ = theta;
    float etheta = theta_r_ - theta;
    if(dtheta < .2)
    {
        Int_theta_ += etheta*dt;
    }
    // Calculate force
    float F_out = P_theta_*etheta - D_theta_*dtheta + I_theta_*Int_theta_ +Fl_;
    // T_out = 0;
    // Mix output
    float left_force =  (T_out/d_ + F_out)/2.0f;
    float right_force = (-T_out/d_ + F_out)/2.0f;

    float l_out = left_force/km_;
    float r_out = right_force/km_;

    if(l_out < 0.)
    {
        l_out = 0.;
    }
    else if(l_out > 1.)
    {
        l_out = 1.;
    }

    if(r_out < 0.)
    {
        r_out = 0.;
    }
    else if(r_out > 1.)
    {
        r_out = 1.;
    }

    hb_msgs::Command command;
    command.left_motor = l_out;
    command.right_motor = r_out;

    command_pub_.publish(command);
}

void PIDController::referenceStateCallback(hb_msgs::ReferenceState::ConstPtr msg)
{
    theta_r_ = msg->pitch;
    psi_r_ = msg->yaw;
}

} // namespace hummingbird_controller
