/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman, Jonathan Bohren
   Desc:   Hardware Interface for any simulated robot in Gazebo
*/

#ifndef _GAZEBO_ROS_CONTROL___DEFAULT_DLR_HIT_HAND_II_HW_SIM_H_
#define _GAZEBO_ROS_CONTROL___DEFAULT_DLR_HIT_HAND_II_HW_SIM_H_

#include <gazebo_ros_control/default_robot_hw_sim.h>


namespace dlr_hit_hand_ii_hw
{

class DefaultDLRHitHandIIHWSim : public gazebo_ros_control::DefaultRobotHWSim {
public:
      virtual bool initSim(
        const std::string& robot_namespace,
        ros::NodeHandle model_nh,
        gazebo::physics::ModelPtr parent_model,
        const urdf::Model *const urdf_model,
        std::vector<transmission_interface::TransmissionInfo> transmissions);


      virtual void writeSim(ros::Time time, ros::Duration period);
};

typedef boost::shared_ptr<DefaultDLRHitHandIIHWSim> DefaultDLRHitHandIIHWSimPtr;

}

#endif // #ifndef __GAZEBO_ROS_CONTROL_PLUGIN_DEFAULT_ROBOT_HW_SIM_H_
