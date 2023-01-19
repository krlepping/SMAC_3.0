#ifndef __LCSR_ASSEMBLY_ASSEMBLY_SIM_MODELS_H__
#define __LCSR_ASSEMBLY_ASSEMBLY_SIM_MODELS_H__

#include <vector>
#include <iterator>
#include <queue>

#include <kdl/frames.hpp>
#include "util.h"

namespace assembly_sim {

  struct MateModel;
  struct AtomModel;

  struct MateFactoryBase;

  struct Mate;
  struct MatePoint;
  struct Atom;

  typedef std::shared_ptr<MateModel> MateModelPtr;
  typedef std::shared_ptr<AtomModel> AtomModelPtr;

  typedef std::shared_ptr<Mate> MatePtr;
  typedef std::shared_ptr<MatePoint> MatePointPtr;
  typedef std::shared_ptr<MateFactoryBase> MateFactoryBasePtr;
  typedef std::shared_ptr<Atom> AtomPtr;

  // The model for a type of mate
  struct MateModel
  {
    MateModel(
        std::string type_,
        sdf::ElementPtr mate_elem_) :
      type(type_),
      mate_elem(mate_elem_)
    {
      // Get the mate template joint
      joint_template_sdf = std::make_shared<sdf::SDF>();
      sdf::init(sdf::SDFPtr(joint_template_sdf));
      sdf::readString(complete_sdf(mate_elem->GetElement("joint")->ToString("")), joint_template_sdf);
      joint_template = joint_template_sdf->Root()->GetElement("model")->GetElement("joint");

      //gzmsg<<"Creating mate model with joint template:"<<std::endl;
      //gzmsg<<joint_template_sdf->ToString()<<std::endl<<std::endl;

      // Get the mate symmetries
      sdf::ElementPtr symmetry_elem = mate_elem->GetElement("symmetry");
      if(symmetry_elem)
      {
        sdf::ElementPtr rot_elem = symmetry_elem->GetElement("rot");

        if(rot_elem)
        {
          ignition::math::Vector3d rot_symmetry;
          rot_elem->GetValue()->Get(rot_symmetry);

          // compute symmetries
          const double x_step = M_PI*2.0/rot_symmetry.X();
          const double y_step = M_PI*2.0/rot_symmetry.Y();
          const double z_step = M_PI*2.0/rot_symmetry.Z();

          for(double ix=0; ix < rot_symmetry.X(); ix++)
          {
            KDL::Rotation Rx = KDL::Rotation::RotX(ix * x_step);
            for(double iy=0; iy < rot_symmetry.Y(); iy++)
            {
              KDL::Rotation Ry = KDL::Rotation::RotY(iy * y_step);
              for(double iz=0; iz < rot_symmetry.Z(); iz++)
              {
                KDL::Rotation Rz = KDL::Rotation::RotZ(iz * z_step);
                symmetries.push_back(KDL::Frame(Rx*Ry*Rz, KDL::Vector(0,0,0)));
              }
            }
          }
        }
      }

      // Add the identity if no symmetries were added
      if(symmetries.size() == 0) {
        symmetries.push_back(KDL::Frame::Identity());
      }
    }

    std::string type;

    // Transforms from the base mate frame to alternative frames
    std::vector<KDL::Frame> symmetries;

    // The mate sdf parameters
    sdf::ElementPtr mate_elem;

    // The sdf template for the joint to be created
    std::shared_ptr<sdf::SDF> joint_template_sdf;
    sdf::ElementPtr joint_template;
  };

  struct MateFactoryBase
  {
    virtual MatePtr createMate(
        MatePointPtr female_mate_point,
        MatePointPtr male_mate_point,
        AtomPtr female_atom,
        AtomPtr male_atom) = 0;
  };

  template <class MateType>
    struct MateFactory : public MateFactoryBase
  {
    MateFactory(
        MateModelPtr mate_model_,
        gazebo::physics::ModelPtr gazebo_model_) :
      mate_model(mate_model_),
      gazebo_model(gazebo_model_)
    {}

    MateModelPtr mate_model;
    gazebo::physics::ModelPtr gazebo_model;

    virtual MatePtr createMate(
        MatePointPtr female_mate_point,
        MatePointPtr male_mate_point,
        AtomPtr female_atom,
        AtomPtr male_atom)
    {
      return std::make_shared<MateType>(
          mate_model,
          gazebo_model,
          female_mate_point,
          male_mate_point,
          female_atom,
          male_atom);
    }
  };

  // This is a male or female point at which a given mate is located
  struct MatePoint
  {
    // The mate model used by this mate point
    MateModelPtr model;
    // Mate point index
    size_t id;
    // The pose of the mate point in the owner frame
    KDL::Frame pose;
  };

  // An instantiated mate
  struct Mate
  {
    enum State {
      NONE = 0, // No state / undefined
      UNMATED = 1, // There is no interaction between this mate's atoms
      MATING = 2, // This mate's atoms are connected by the transitional mechanism
      MATED = 3, // This mate's atoms are connected by a static joint
      SUPPRESSED = 4 // This mate should be ignored, if already mated it should be demated
    };

    Mate(
      MateModelPtr mate_model,
      gazebo::physics::ModelPtr gazebo_model_,
      MatePointPtr female_mate_point_,
      MatePointPtr male_mate_point_,
      AtomPtr female_atom,
      AtomPtr male_atom);

    // Determine if the mate's attachment state needs to be updated
    // Asynchronous with Gazebo's update loop
    virtual void queueUpdate() = 0;

    // Update the constraint connectivity
    virtual void updateConstraints() = 0;

    // Update calculations needed to be done every tick
    virtual void update(gazebo::common::Time timestep) = 0;

    // Update functions
    void requestUpdate(State new_pending_state) { pending_state = new_pending_state; }
    bool needsUpdate() const { return pending_state != NONE; }
    void serviceUpdate() { pending_state = NONE; }
    State getUpdate() { return pending_state; }

    // Suppress function
    void suppressMate(bool suppress) {
        suppress_requested = suppress;
    }

    // Introspection
    std::string description;
    std::string getDescription() {
      return description;
    }

    virtual void getMarkers(visualization_msgs::MarkerArray &marker_array) {}

    // Mate model (same as mate points)
    MateModelPtr model;

    // Attachment states
    Mate::State state, pending_state;

    // Joint SDF
    sdf::ElementPtr joint_sdf;
    // Joint associated with mate
    // If this is NULL then the mate is unoccupied
    gazebo::physics::JointPtr joint;
    gazebo::physics::ModelPtr gazebo_model;

    // Atoms associated with this mate
    AtomPtr female;
    AtomPtr male;

    // Mate points
    MatePointPtr female_mate_point;
    MatePointPtr male_mate_point;

    // Mate error from female to male (including symmetries)
    KDL::Twist mate_error;
    // Mate point error
    KDL::Twist mate_point_error;

    std::vector<KDL::Frame>::iterator mated_symmetry;

    // Max erp
    double max_stop_erp;
    double max_erp;

    // Asynchronous request that the mate be suppressed
    bool suppress_requested;
  };

  // The model for a type of atom
  struct AtomModel
  {
    // The type of atom
    std::string type;

    // Models for the mates
    std::vector<MatePointPtr> female_mate_points;
    std::vector<MatePointPtr> male_mate_points;

    // The sdf for the link to be created for this atom
    std::shared_ptr<sdf::SDF> link_template_sdf;
    sdf::ElementPtr link_template;
  };

  // An instantiated atom
  struct Atom
  {
    // The model used by this atom
    AtomModelPtr model;

    // The link on the assembly model
    gazebo::physics::LinkPtr link;
    KDL::Wrench wrench;
  };

  // Get links connected to a given root link
  void GetConnectedLinks(
      gazebo::physics::LinkPtr root_link,
      boost::unordered_set<gazebo::physics::LinkPtr> &connected_component,
      bool &connected_component_is_static);

  // A mate
  struct ProximityMateBase : public Mate
  {
    // Threshold for attaching a mate
    double attach_threshold_linear;
    double attach_threshold_angular;

    // Threshold for detaching a mate
    double detach_threshold_linear;
    double detach_threshold_angular;

    Eigen::Vector3d max_force, max_torque;

    ProximityMateBase(
        MateModelPtr mate_model,
        gazebo::physics::ModelPtr gazebo_model,
        MatePointPtr female_mate_point_,
        MatePointPtr male_mate_point_,
        AtomPtr female_atom,
        AtomPtr male_atom) :
      Mate(mate_model, gazebo_model, female_mate_point_, male_mate_point_, female_atom, male_atom),
      max_force(Eigen::Vector3d::Zero()),
      max_torque(Eigen::Vector3d::Zero())
    {
      this->load_proximity_params();
    }

  protected:
    virtual void load_proximity_params()
    {
      sdf::ElementPtr mate_elem = model->mate_elem;

      // Get the attach/detach thresholds
      sdf::ElementPtr attach_threshold_elem = mate_elem->GetElement("attach_threshold");
      if(attach_threshold_elem and attach_threshold_elem->HasElement("linear") and attach_threshold_elem->HasElement("angular"))
      {
        attach_threshold_elem->GetElement("linear")->GetValue()->Get(attach_threshold_linear);
        attach_threshold_elem->GetElement("angular")->GetValue()->Get(attach_threshold_angular);

        //gzwarn<<(boost::format("Attach threshold linear: %f angular %f") % attach_threshold_linear % attach_threshold_angular)<<std::endl;
      } else {
        gzerr<<"No attach_threshold / linear / angular elements!"<<std::endl;
      }

      sdf::ElementPtr detach_threshold_elem = mate_elem->GetElement("detach_threshold");
      if(detach_threshold_elem and detach_threshold_elem->HasElement("linear") and detach_threshold_elem->HasElement("angular"))
      {
        detach_threshold_elem->GetElement("linear")->GetValue()->Get(detach_threshold_linear);
        detach_threshold_elem->GetElement("angular")->GetValue()->Get(detach_threshold_angular);

        //gzwarn<<(boost::format("Detach threshold linear: %f angular %f") % attach_threshold_linear % attach_threshold_angular)<<std::endl;
      } else {
        gzerr<<"No detach_threshold / linear / angular elements!"<<std::endl;
      }

      ignition::math::Vector3d gz_max_force, gz_max_torque;
      sdf::ElementPtr max_force_elem = mate_elem->GetElement("max_force");
      if(max_force_elem) {
        max_force_elem->GetValue()->Get(gz_max_force);
        to_eigen(gz_max_force, max_force);
      }
      sdf::ElementPtr max_torque_elem = mate_elem->GetElement("max_torque");
      if(max_torque_elem) {
        max_torque_elem->GetValue()->Get(gz_max_torque);
        to_eigen(gz_max_torque, max_torque);
      }
    }

    virtual void attach()
    {
      // TODO:
      // mate_point.M * this->mate_error // this will rotate the base frame

      // TODO: Set pose for all other links attached to the jumped link
      // - get jump offset
      // - apply jump to each element in the connected component that this link is in
      // - merge connected components

      // Get connected components
      boost::unordered_set<gazebo::physics::LinkPtr>
        female_component,
        male_component;
      bool
        female_component_static,
        male_component_static;

      GetConnectedLinks(this->female->link, female_component, female_component_static);
      GetConnectedLinks(this->male->link, male_component, male_component_static);

      // detach the atoms if they're already attached (they're going to be re-attached)
      this->detach();

      // If the components aren't connected then, one of them needs to be jumped into place
      if(female_component.find(this->male->link) == female_component.end()) {
        // Get the atom frames
        KDL::Frame female_atom_frame, male_atom_frame;
        to_kdl(this->female->link->WorldPose(), female_atom_frame);
        to_kdl(this->male->link->WorldPose(), male_atom_frame);

        // move one of the atoms into the precise relative position
        KDL::Frame current_root_frame;
        KDL::Frame final_root_frame;

        // Component iterator
        boost::unordered_set<gazebo::physics::LinkPtr>::iterator it_l_begin(NULL), it_l_end(NULL);

        if(!male_component_static) {
          // Define the current and final root frames for moving the male atom
          current_root_frame = male_atom_frame;
          final_root_frame = female_atom_frame * female_mate_point->pose * (*this->mated_symmetry) * (this->male_mate_point->pose).Inverse();

          // Set iterator to male component
          it_l_begin = male_component.begin();
          it_l_end = male_component.end();
        } else if(!female_component_static) {
          // Define the current and final root frames for moving the female atom
          current_root_frame = female_atom_frame;
          final_root_frame = male_atom_frame * male_mate_point->pose * (*this->mated_symmetry).Inverse() * (this->female_mate_point->pose).Inverse();

          // Set iterator to female component
          it_l_begin = female_component.begin();
          it_l_end = female_component.end();
        }

        // Define the fixed-frame component transform
        KDL::Frame component_transform = final_root_frame * current_root_frame.Inverse();

        KDL::Frame current_component_frame;
        KDL::Frame final_component_frame;
        ignition::math::Pose3d final_component_pose;

        for(boost::unordered_set<gazebo::physics::LinkPtr>::iterator it_l = it_l_begin; it_l != it_l_end; ++it_l) {
          gazebo::physics::LinkPtr comp_link =  *it_l;

          // Move component
          to_kdl(comp_link->WorldPose(), current_component_frame);
          final_component_frame = component_transform * current_component_frame;
          to_gazebo(final_component_frame, final_component_pose);
          comp_link->SetWorldPose(final_component_pose);

          //gzwarn<<"jumping "<<comp_link->GetName()<<std::endl;
        }
      }

      // attach two atoms via joint
      this->joint->Attach(this->female->link, this->male->link);

      // Male and female mate frames in world coordinates
      //gzwarn<<">> mate error (before): "<<this->mate_error.vel.Norm()<<", "<<this->mate_error.rot.Norm()<<std::endl;
      KDL::Frame female_atom_frame, male_atom_frame;
      KDL::Frame female_mate_frame, male_mate_frame;
      to_kdl(this->female->link->WorldPose(), female_atom_frame);
      to_kdl(this->male->link->WorldPose(), male_atom_frame);
      female_mate_frame = female_atom_frame * female_mate_point->pose * (*this->mated_symmetry);
      male_mate_frame = male_atom_frame * male_mate_point->pose;
      this->mate_error = KDL::diff(female_mate_frame, male_mate_frame);
      //gzwarn<<">> mate error (after):  "<<this->mate_error.vel.Norm()<<", "<<this->mate_error.rot.Norm()<<std::endl;

#if 0
      // set stiffness based on proximity to goal
      double lin_err = this->mate_error.vel.Norm();
      double ang_err = this->mate_error.rot.Norm();
      double max_lin_err = detach_threshold_linear;
      double max_ang_err = detach_threshold_angular;
      //this->joint->SetAttribute(
      //"stop_erp", 0,
      //this->max_stop_erp *
      //std::min(std::max(0.0, max_lin_err - lin_err), max_lin_err) / max_lin_err *
      //std::min(std::max(0.0, max_ang_err - ang_err), max_ang_err) / max_ang_err);
      //this->joint->SetAttribute(
      //"erp", 0,
      //this->max_erp *
      //std::min(std::max(0.0, max_lin_err - lin_err), max_lin_err) / max_lin_err *
      //std::min(std::max(0.0, max_ang_err - ang_err), max_ang_err) / max_ang_err);

      //this->joint->SetHighStop(0, lin_err);

      // get the location of the joint in the child (male atom) frame, as specified by the SDF
      KDL::Frame initial_anchor_frame, actual_anchor_frame;
      to_kdl(this->joint->InitialAnchorPose(), initial_anchor_frame);
      actual_anchor_frame = male_atom_frame*initial_anchor_frame;

      // Set the anchor position (location of the joint)
      // This is in the WORLD frame
      // IMPORTANT: This avoids injecting energy into the system in the form of a constraint violation
      ignition::math::Pose3d actual_anchor_pose;
      to_gazebo(actual_anchor_frame, actual_anchor_pose);
      this->joint->SetAnchor(0, actual_anchor_pose.Pos());

      // Save the anchor offset (mate point to anchor)
      this->anchor_offset = (
          actual_anchor_frame.Inverse() *   // anchor to world
          male_atom_frame *                 // world to atom
          this->male_mate_point->pose // atom to mate point
          ).Inverse();
      //gzwarn<<" ---- initial anchor pose: "<<std::endl<<initial_anchor_frame<<std::endl;
      //gzwarn<<" ---- actual anchor pose: "<<std::endl<<actual_anchor_frame<<std::endl;
#endif
    }

    virtual void detach()
    {
      // Simply detach joint
      joint->Detach();
    }
  };

  struct ProximityMate : public ProximityMateBase
  {
    ProximityMate(
        MateModelPtr mate_model,
        gazebo::physics::ModelPtr gazebo_model,
        MatePointPtr female_mate_point_,
        MatePointPtr male_mate_point_,
        AtomPtr female_atom,
        AtomPtr male_atom) :
      ProximityMateBase(mate_model, gazebo_model, female_mate_point_, male_mate_point_, female_atom, male_atom)
    {
    }

    virtual void queueUpdate()
    {
      // Convenient references
      AtomPtr &female_atom = female;
      AtomPtr &male_atom = male;

      // Only analyze mates with associated joints
      if(!joint) {
        gzwarn<<"No joint for mate from "<<female_atom->link->GetName()<<" -> "<<male_atom->link->GetName()<<std::endl;
        return;
      }

      // Handle suppress request
      if (suppress_requested && state != Mate::SUPPRESSED)
      {
        this->requestUpdate(Mate::SUPPRESSED);
        //gzwarn<<"Suppressing mate: "<<getDescription()<<std::endl;
        return;
      }
      else if (!suppress_requested && state == Mate::SUPPRESSED)
      {
        this->requestUpdate(Mate::UNMATED);
        gzwarn<<"Reactivating mate: "<<getDescription()<<std::endl;
        return;
      }

      // Don't do any logic this mate is suppressed
      if(state == Mate::SUPPRESSED) {
        return;
      }

      KDL::Frame female_atom_frame;
      to_kdl(female_atom->link->WorldPose(), female_atom_frame);

      KDL::Frame male_atom_frame;
      to_kdl(male_atom->link->WorldPose(), male_atom_frame);

      // Iterate over all symmetric mating positions
      for(std::vector<KDL::Frame>::iterator it_sym = model->symmetries.begin();
          it_sym != model->symmetries.end();
          ++it_sym)
      {
        // Compute the world frame of the female mate frame
        // This takes into account symmetries in the mate
        KDL::Frame female_mate_frame = female_atom_frame * female_mate_point->pose * (*it_sym);

        // Compute the world pose of the male mate frame
        KDL::Frame male_mate_frame = male_atom_frame * male_mate_point->pose;

        // compute twist between the two mate points
        KDL::Twist twist_err = diff(female_mate_frame, male_mate_frame);
        //gzwarn<<female_mate_point->pose.M<<std::endl;
        //gzwarn<<twist_err.vel.Norm()<<", "<<twist_err.rot.Norm()<<" VS "<<this->mate_error.vel.Norm()<<", "<<this->mate_error.rot.Norm()<<std::endl;

        if(state == Mate::MATED and it_sym == mated_symmetry)
        {
          gazebo::physics::JointWrench joint_wrench = joint->GetForceTorque(0);
          Eigen::Vector3d force, torque;
          to_eigen(joint_wrench.body1Force, force);
          to_eigen(joint_wrench.body1Torque, torque);

          //gzwarn<<">>> "<<this->getDescription()<<" Force: "<<force<<" Torque: "<<torque<<std::endl;
          // Determine if active mate needs to be detached
          if(twist_err.vel.Norm() > detach_threshold_linear or
             twist_err.rot.Norm() > detach_threshold_angular or
             ((max_force.array() > 0.0).any() and (force.array().abs() > max_force.array()).any()) or
             ((max_torque.array() > 0.0).any() and (torque.array().abs() > max_torque.array()).any()))
          {
            // The mate points are beyond the detach threhold and should be demated
            gzwarn<<"> Request unmate "<<getDescription()<<std::endl;
            gzwarn<<">    linear error: "<<twist_err.vel.Norm()<<std::endl;
            gzwarn<<">   angular error: "<<twist_err.vel.Norm()<<std::endl;
            this->requestUpdate(Mate::UNMATED);
            break;
          } else if(
              twist_err.vel.Norm() / mate_error.vel.Norm() < 0.8 and
              twist_err.rot.Norm() / mate_error.rot.Norm() < 0.8)
          {
            // Re-mate but closer to the desired point
            gzwarn<<"> Request remate "<<getDescription()<<std::endl;
            this->mated_symmetry = it_sym;
            this->mate_error = twist_err;
            this->requestUpdate(Mate::MATED);
            break;
          }
        } else {
          // Determine if mated atoms need to be attached
          if(twist_err.vel.Norm() < attach_threshold_linear and
             twist_err.rot.Norm() < attach_threshold_angular)
          {
            // The mate points are within the attach threshold and should be mated
            //gzwarn<<"> Request mate "<<getDescription()<<std::endl;
            //gzwarn<<">    linear error: "<<twist_err.vel.Norm()<<std::endl;
            //gzwarn<<">   angular error: "<<twist_err.vel.Norm()<<std::endl;
            this->mated_symmetry = it_sym;
            this->mate_error = twist_err;
            this->requestUpdate(Mate::MATED);
            break;
          }
        }
      }
    }

    virtual void updateConstraints()
    {
      if(not this->needsUpdate()) {
        this->queueUpdate();
      }

      if(not this->needsUpdate()) {
        return;
      }

      State new_pending_state = this->getUpdate();

      switch(new_pending_state) {
        case Mate::NONE:
          break;

        case Mate::SUPPRESSED:
          if((this->state == Mate::MATED) || (this->state == Mate::MATING)) {
            //gzwarn<<"> Suppressing and deatching an active mate: "<<getDescription()<<std::endl;
            this->detach();
            this->state = Mate::SUPPRESSED;
            this->mated_symmetry = model->symmetries.end();
          } else {
            //gzwarn<<"> Suppressing an inactive mate: "<<getDescription()<<std::endl;
            this->state = Mate::SUPPRESSED;
          }
          break;

        case Mate::UNMATED:
        case Mate::MATING:
          if(state == Mate::MATED) {
            gzwarn<<"> Detaching "<<female->link->GetName()<<" from "<<male->link->GetName()<<"!"<<std::endl;
            this->detach();
            this->state = Mate::UNMATED;
            this->mated_symmetry = model->symmetries.end();
          } else if(state == Mate::SUPPRESSED)
          {
            gzwarn<<"> Unsuppressing: "<<getDescription()<<std::endl;
            this->state = new_pending_state;
          }
          break;

        case Mate::MATED:
          if(state != Mate::MATED) {
            //gzwarn<<"> Attaching "<<female->link->GetName()<<" to "<<male->link->GetName()<<"!"<<std::endl;
          } else {
            gzwarn<<"> Reattaching "<<female->link->GetName()<<" to "<<male->link->GetName()<<"!"<<std::endl;
          }
          this->attach();
          this->state = Mate::MATED;
          break;
      };

      this->serviceUpdate();
    }

    virtual void update(gazebo::common::Time timestep)
    {
    }

  };

  struct DipoleMate : public ProximityMate
  {
    // Individual magnetic dipole parameters
    struct Dipole {
      double min_distance;
      KDL::Vector position;
      KDL::Vector moment;
    };

    // Dipoles involved in this mate
    std::vector<Dipole> dipoles;

    DipoleMate(
        MateModelPtr mate_model,
        gazebo::physics::ModelPtr gazebo_model,
        MatePointPtr female_mate_point_,
        MatePointPtr male_mate_point_,
        AtomPtr female_atom,
        AtomPtr male_atom) :
      ProximityMate(mate_model, gazebo_model, female_mate_point_, male_mate_point_, female_atom, male_atom)
    {
      this->load();
    }

    double max_distance;
    std::vector<visualization_msgs::Marker> female_moment_markers;
    std::vector<visualization_msgs::Marker> male_moment_markers;

    virtual void getMarkers(visualization_msgs::MarkerArray &marker_array)
    {
      // Only add moment markers if this mate is activated
      if(state == Mate::MATED or mate_point_error.vel.Norm() > max_distance) {
        return;
      }
      std::copy(female_moment_markers.begin(), female_moment_markers.end(), std::back_inserter(marker_array.markers));
      std::copy(male_moment_markers.begin(), male_moment_markers.end(), std::back_inserter(marker_array.markers));
    }

    virtual void load()
    {
      sdf::ElementPtr mate_elem = model->mate_elem;

      // Get maximum distance
      sdf::ElementPtr max_dist_elem = mate_elem->GetElement("max_distance");
      max_dist_elem->GetValue()->Get(max_distance);

      // Get the dipole moments (along Z axis)
      sdf::ElementPtr dipole_elem = mate_elem->GetElement("dipole");

      while(dipole_elem && dipole_elem->GetName() == "dipole")
      {
        ignition::math::Vector3d position_gz, moment_gz;
        double min_distance;

        // Get the position of the dipole
        sdf::ElementPtr position_elem = dipole_elem->GetElement("position");
        position_elem->GetValue()->Get(position_gz);

        // Get the magnetic moment
        sdf::ElementPtr moment_elem = dipole_elem->GetElement("moment");
        moment_elem->GetValue()->Get(moment_gz);

        // Get minimum distance
        sdf::ElementPtr min_dist_elem = dipole_elem->GetElement("min_distance");
        min_dist_elem->GetValue()->Get(min_distance);

        Dipole dipole;
        dipole.position = KDL::Vector(position_gz.X(), position_gz.Y(), position_gz.Z());
        dipole.moment = KDL::Vector(moment_gz.X(), moment_gz.Y(), moment_gz.Z());
        dipole.min_distance = min_distance;

        dipoles.push_back(dipole);

        // Add marker
        visualization_msgs::Marker m;
        m.ns = description+"/female";
        m.id = female_moment_markers.size();
        m.header.frame_id = "/world";
        m.header.stamp = ros::Time::now();
        m.type = visualization_msgs::Marker::ARROW;
        m.scale.x = 0.005;
        m.scale.y = 0.01;
        m.scale.z = 0.01;
        m.color.a = 0.5;
        m.color.g = 1.0;
        geometry_msgs::Point p0, p1;
        m.points.push_back(p0);
        m.points.push_back(p1);
        female_moment_markers.push_back(m);
        m.ns = description+"/male";
        m.color.b = 1.0;
        m.color.g = 0.0;
        male_moment_markers.push_back(m);

        // Get the next dipole element
        dipole_elem = dipole_elem->GetNextElement(dipole_elem->GetName());
      }
    }

    virtual void update(gazebo::common::Time timestep)
    {
      // Convenient references
      AtomPtr &female_atom = this->female;
      AtomPtr &male_atom = this->male;

      MatePointPtr &female_mate_point = this->female_mate_point;
      MatePointPtr &male_mate_point = this->male_mate_point;

      // Don't apply magnetic force if the mate is attached
      if(state == Mate::MATED) {
        return;
      }

      // Don't apply magnetic force if the mate is suppressed
      if(state == Mate::SUPPRESSED)
      {
        return;
      }

      // Compute the world pose of the female mate frame
      KDL::Frame female_atom_frame;
      to_kdl(female_atom->link->WorldPose(), female_atom_frame);
      KDL::Frame female_mate_frame = female_atom_frame * female_mate_point->pose;
      ignition::math::Pose3d female_mate_pose;
      to_gazebo(female_mate_frame, female_mate_pose);

      // Compute the world pose of the male mate frame
      KDL::Frame male_atom_frame;
      to_kdl(male_atom->link->WorldPose(), male_atom_frame);
      KDL::Frame male_mate_frame = male_atom_frame * male_mate_point->pose;
      ignition::math::Pose3d male_mate_pose;
      to_gazebo(male_mate_frame, male_mate_pose);

      // compute twist between the two mate points to determine if we need to simulate dipole interactions
      mate_point_error = diff(female_mate_frame, male_mate_frame);
      if(mate_point_error.vel.Norm() > max_distance) {
        return;
      } else {
        //gzwarn<<"mate "<<description<<" attracting at "<<mate_point_error.vel<<"meters"<<std::endl;
      }

      // Working variables
      KDL::Vector r;
      KDL::Vector rh;
      KDL::Frame female_dipole_frame, male_dipole_frame;
      KDL::Twist twist_err;
      KDL::Vector m1, m2;
      KDL::Vector B1, B2;
      KDL::Wrench W1, W2;
      ignition::math::Vector3d F1gz, F2gz, T1gz, T2gz;

      // update markers
      int marker_id = 0;
      for(std::vector<Dipole>::iterator it_fdp=dipoles.begin(); it_fdp!=dipoles.end(); ++it_fdp)
      {
        female_dipole_frame.p = female_mate_frame*it_fdp->position;
        female_dipole_frame.M = female_mate_frame.M;
        m1 = female_dipole_frame.M * it_fdp->moment;
        visualization_msgs::Marker &m = female_moment_markers[marker_id];
        m.points[0].x = female_dipole_frame.p.x();
        m.points[0].y = female_dipole_frame.p.y();
        m.points[0].z = female_dipole_frame.p.z();
        m.points[1].x = female_dipole_frame.p.x() + m1.x();
        m.points[1].y = female_dipole_frame.p.y() + m1.y();
        m.points[1].z = female_dipole_frame.p.z() + m1.z();
        marker_id++;
      }
      marker_id = 0;
      for(std::vector<Dipole>::iterator it_mdp=dipoles.begin(); it_mdp!=dipoles.end(); ++it_mdp)
      {
        male_dipole_frame.p = male_mate_frame*it_mdp->position;
        male_dipole_frame.M = male_mate_frame.M;
        m2 = male_dipole_frame.M * it_mdp->moment;
        visualization_msgs::Marker &m = male_moment_markers[marker_id];
        m.points[0].x = male_dipole_frame.p.x();
        m.points[0].y = male_dipole_frame.p.y();
        m.points[0].z = male_dipole_frame.p.z();
        m.points[1].x = male_dipole_frame.p.x() + m1.x();
        m.points[1].y = male_dipole_frame.p.y() + m1.y();
        m.points[1].z = male_dipole_frame.p.z() + m1.z();
        marker_id++;
      }

      // Compute dipole force
      static const double mu0 = 4*M_PI*1E-7;

      // compute and apply forces between all male/female pairs of dipoles
      for(std::vector<Dipole>::iterator it_fdp=dipoles.begin(); it_fdp!=dipoles.end(); ++it_fdp)
      {
        // Compute the moment orientation and position
        female_dipole_frame.M = female_mate_frame.M;
        female_dipole_frame.p = female_mate_frame*it_fdp->position;
        ignition::math::Pose3d female_dipole_pose;
        to_gazebo(female_dipole_frame, female_dipole_pose);
        m1 = female_dipole_frame.M * it_fdp->moment;

        for(std::vector<Dipole>::iterator it_mdp=dipoles.begin(); it_mdp!=dipoles.end(); ++it_mdp)
        {
          // Compute the moment orientation and position
          male_dipole_frame.M = male_mate_frame.M;
          male_dipole_frame.p = male_mate_frame*it_mdp->position;
          ignition::math::Pose3d male_dipole_pose;
          to_gazebo(male_dipole_frame, male_dipole_pose);
          m2 = male_dipole_frame.M * it_mdp->moment;

          // compute twist between the two mate points
          twist_err = diff(female_dipole_frame, male_dipole_frame);

          r = twist_err.vel;
          double rn = r.Norm();
          rn = std::max(it_fdp->min_distance, rn);
          r = rn / r.Norm() * r;

          // Compute normalized distance
          rh = (rn > 1E-5) ? twist_err.vel/rn : KDL::Vector(1,0,0);

          // Compute magnetic fields
          B1 = mu0 / 4 / M_PI / pow(rn,3) * ( 3 * (KDL::dot(m1,rh)*rh - m1));
          B2 = mu0 / 4 / M_PI / pow(rn,3) * ( 3 * (KDL::dot(m2,rh)*rh - m2));

          // Compute wrenches in the world frame applied at the dipole point
          W1.force = -3 * mu0 / 4 / M_PI / pow(rn,4) * ( (rh*m2)*m1 + (rh*m1)*m2 - 2*rh*KDL::dot(m1,m2) + 5*rh*KDL::dot(rh*m2,rh*m1) );
          W2.force = -W1.force;
          W1.torque = m1 * B2;
          W2.torque = m2 * B1;

          //mate_point_error.vel = W1.force;
          //mate_point_error.rot = W1.torque;
#if 0
          // Convert to wrenches applied at centers of mass
          KDL::Wrench W1cog(W1), W2cog(W2);
          W1cog.force -= W1.torque * it_fdp->position / pow(it_fdp->position.Norm(),2.0);
          W2cog.force -= W2.torque * it_mdp->position / pow(it_mdp->position.Norm(),2.0);
#endif
#if 0
          gzwarn<<"Dipole force: "<<std::setprecision(4)<<std::fixed
            <<"female("<<female_atom->link->GetName()<<"#"<<female_mate_point->id<<") "<<female_mate_frame<<" --> "<<m1<<" F1="<<W1.force
            <<" - "
            <<"male("<<male_atom->link->GetName()<<"#"<<male_mate_point->id<<") "<<male_mate_frame<<" --> "<<m2<<" F2="<<W2.force
            <<""<<std::endl;
#endif
          // Apply force to links
          to_gazebo(W1, F1gz, T1gz);
          to_gazebo(W2, F2gz, T2gz);

          female_atom->link->AddForceAtWorldPosition(F1gz, female_dipole_pose.Pos());
          female_atom->link->AddTorque(T1gz);
          female_atom->wrench += W1;

          male_atom->link->AddForceAtWorldPosition(F2gz, male_dipole_pose.Pos());
          male_atom->link->AddTorque(T2gz);
          male_atom->wrench += W2;
        }
      }
    }
  };
}

#endif // ifndef __LCSR_ASSEMBLY_ASSEMBLY_SIM_MOgazebo::common::Time timestepDELS_H__
