#ifndef __LCSR_ASSEMBLY_ASSEMBLY_SIM_UTIL_H__
#define __LCSR_ASSEMBLY_ASSEMBLY_SIM_UTIL_H__

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <boost/make_shared.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

// tf headers
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>

// visualize mates in rviz gui
#include <visualization_msgs/MarkerArray.h>

// send information to ros
#include <assembly_msgs/MateList.h>

#include <kdl/frames_io.hpp>

namespace assembly_sim {
  // helper function to create names for TF
  std::string getNameTF(const std::string &ns, const std::string &joint);

  // Convert geometric types
  void to_kdl(const sdf::ElementPtr &pose_elem, KDL::Frame &frame);

  void to_kdl(const ignition::math::Pose3d &pose, KDL::Frame &frame);

  void to_kdl(const ignition::math::Vector3d &vector3, KDL::Vector &vector);

  void to_tf(const ignition::math::Pose3d &pose, tf::Transform &frame);

  void to_gazebo(const KDL::Frame &frame, ignition::math::Pose3d &pose);

  void to_gazebo(const KDL::Wrench &wrench, ignition::math::Vector3d &force, ignition::math::Vector3d &torque);

  void to_eigen(const ignition::math::Vector3d &vector3, Eigen::Vector3d &vector3d);

  // Complete an SDF xml snippet into a model
  std::string complete_sdf(const std::string &incomplete_sdf);
}

#endif // ifndef __LCSR_ASSEMBLY_ASSEMBLY_SIM_UTIL_H__
