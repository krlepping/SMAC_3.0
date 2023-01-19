#include "util.h"

namespace assembly_sim {
  /************************************************************************************/
  /*                                Helper Functions                                  */
  /************************************************************************************/

  // helper function to create names for TF
  std::string getNameTF(const std::string &ns, const std::string &joint)
  {
    std::stringstream ss;
    ss << ns << "/" << joint;
    return ss.str();
  }

  // Convert pose types
  void to_kdl(const sdf::ElementPtr &pose_elem, KDL::Frame &frame)
  {
    ignition::math::Pose3d pose;
    pose_elem->GetValue()->Get(pose);

    //std::string pose_str;
    //pose_elem->GetValue()->Get(pose_str);

    frame = KDL::Frame(
        KDL::Rotation::Quaternion(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W()),
        KDL::Vector(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()));

    //gzwarn<<"string of joint pose: "<<pose_str<<std::endl<<frame<<std::endl;
  }

  void to_kdl(const ignition::math::Pose3d &pose, KDL::Frame &frame)
  {
    frame = KDL::Frame(
        KDL::Rotation::Quaternion(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W()),
        KDL::Vector(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()));
  }

  void to_kdl(const ignition::math::Vector3d &vector3, KDL::Vector &vector)
  {
    vector.data[0] = vector3.X();
    vector.data[1] = vector3.Y();
    vector.data[2] = vector3.Z();
  }

  void to_tf(const ignition::math::Pose3d &pose, tf::Transform &frame)
  {
    frame.setRotation( tf::Quaternion(
            pose.Rot().X(),
            pose.Rot().Y(),
            pose.Rot().Z(),
            pose.Rot().W()));
    frame.setOrigin( tf::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()) );
  }

  void to_gazebo(const KDL::Frame &frame, ignition::math::Pose3d &pose)
  {
    pose = ignition::math::Pose3<double>(
        ignition::math::Vector3<double>(frame.p.data[0], frame.p.data[1], frame.p.data[2]),
        ignition::math::Quaternion<double>());
    frame.M.GetQuaternion(pose.Rot().X(),
                          pose.Rot().Y(),
                          pose.Rot().Z(),
                          pose.Rot().W());
  }

  void to_gazebo(const KDL::Wrench &wrench, ignition::math::Vector3d &force, ignition::math::Vector3d &torque)
  {
    force.X(wrench.force.x());
    force.Y(wrench.force.y());
    force.Z(wrench.force.z());

    torque.X(wrench.torque.x());
    torque.Y(wrench.torque.y());
    torque.Z(wrench.torque.z());
  }

  void to_eigen(const ignition::math::Vector3d &vector3, Eigen::Vector3d &vector3d) {
    for(int i=0; i<3; i++) {
      vector3d[i] = vector3[i];
    }
  }

  // Complete an SDF xml snippet into a model
  std::string complete_sdf(const std::string &incomplete_sdf)
  {
    return std::string("<sdf version=\"1.4\">\n<model name=\"template\">\n" + incomplete_sdf + "\n</model>\n</sdf>");
  }

}
