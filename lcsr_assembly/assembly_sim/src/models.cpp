#include <boost/format.hpp>

#include "models.h"

namespace assembly_sim {
  Mate::Mate(
      MateModelPtr mate_model,
      gazebo::physics::ModelPtr gazebo_model_,
      MatePointPtr female_mate_point_,
      MatePointPtr male_mate_point_,
      AtomPtr female_atom,
      AtomPtr male_atom) :
    model(mate_model),
    state(Mate::UNMATED),
    pending_state(Mate::NONE),
    female(female_atom),
    male(male_atom),
    female_mate_point(female_mate_point_),
    male_mate_point(male_mate_point_),
    joint_sdf(),
    joint(),
    gazebo_model(gazebo_model_),
    mated_symmetry(model->symmetries.end()),
    mate_point_error(KDL::Vector(0,0,0),KDL::Vector(0,0,0)),
    suppress_requested(false)
  {
    // Make sure male and female mate points have the same model
    assert(female_mate_point->model == male_mate_point->model);

    //gzwarn<<"Creating joint for mate type: "
    //  <<female_mate_point->model->type<<" ("
    //  <<female_atom->link->GetName()<<"#"
    //  <<female_mate_point->id
    //  <<") -> ("
    //  <<male_atom->link->GetName()<<"#"
    //  <<male_mate_point->id<<")"
    //  <<std::endl;

    description = boost::str(boost::format("%s#%d -> %s#%d")% female_atom->link->GetName()% female_mate_point->id% male_atom->link->GetName()% male_mate_point->id);

    // Get the joint type
    std::string joint_type;
    model->joint_template->GetAttribute("type")->Get(joint_type);
    //gzwarn<<"Creating joint with type: "<<joint_type<<std::endl;

    // Customize the joint sdf template
    joint_sdf = std::make_shared<sdf::Element>();
    joint_sdf->Copy(model->joint_template);
    std::string name = boost::str( boost::format("%s_m%0d_to_%s_m%0d")
             % female_atom->link->GetName()
             % female_mate_point->id
             % male_atom->link->GetName()
             % male_mate_point->id);
    joint_sdf->GetAttribute("name")->Set(name);
    joint_sdf->GetElement("parent")->GetValue()->Set(female_atom->link->GetScopedName());
    joint_sdf->GetElement("child")->GetValue()->Set(male_atom->link->GetScopedName());

    ignition::math::Pose3d pose;
    to_gazebo(male_mate_point->pose, pose);
    joint_sdf->GetElement("pose")->GetValue()->Set(pose);

    //gzwarn<<"joint sdf:\n\n"<<joint_sdf->ToString(">>")<<std::endl;

    // Construct the actual joint between these two atom links
    //gzwarn<<"Creating joint in model: "<<gazebo_model->GetName()<<"..."<<std::endl;
    joint = gazebo_model->CreateJoint(name, joint_type, female_atom->link, male_atom->link);

    // Load joint description from SDF
    //  - sets parend a child links
    //  - sets the anchor pose
    //  - loads sensor elements
    //gzwarn<<"Loading joint properties..."<<std::endl;
    try {
        joint->Load(joint_sdf);
    } catch(...) {
        gzerr<<"Could not load joint."<<std::endl;
        for(auto &link : gazebo_model->GetLinks()) {
            gzerr<<" - "<<link->GetName()<<std::endl;
        }
        for(auto &m : gazebo_model->NestedModels()) {
            gzerr<<" - "<<m->GetName()<<std::endl;
            for(auto &link : m->GetLinks()) {
                gzerr<<"   - "<<link->GetName()<<std::endl;
            }
        }
    }

    // Initialize joint
    //  - sets axis orientation
    //  - sets axis limits
    //  - attaches parent and child via this joint
    //gzwarn<<"Initializing joint..."<<std::endl;
    joint->Init();

    // Joints should initially be detached
    joint->Detach();

    // Get the stop stiffness
    max_erp = joint->GetParam("erp",0);
    max_stop_erp = joint->GetParam("stop_erp",0);

    //gzmsg<<"Joint "<<description<<" created."<<std::endl;
  }

  void GetConnectedLinks(
      gazebo::physics::LinkPtr root_link,
      boost::unordered_set<gazebo::physics::LinkPtr> &connected_component,
      bool &connected_component_is_static)
  {
    std::queue<gazebo::physics::LinkPtr> links_to_check;
    boost::unordered_set<gazebo::physics::LinkPtr> checked_links;

    links_to_check.push(root_link);

    //gzwarn<<"computing component for: "<<root_link->GetName()<<std::endl;

    while(links_to_check.size() > 0) {
      // Get a link from the queue and add it to the component
      gazebo::physics::LinkPtr link = links_to_check.front();
      links_to_check.pop();
      connected_component.insert(link);
      checked_links.insert(link);
      //gzwarn<<" adding link to component: "<<link->GetName()<<std::endl;

      if(link->IsStatic()) {
        connected_component_is_static = true;
      }

      // Add parent links to the queue
      std::vector<gazebo::physics::LinkPtr> parent_links = link->GetParentJointsLinks();
      for(std::vector<gazebo::physics::LinkPtr>::iterator it_l = parent_links.begin();
          it_l != parent_links.end();
          ++it_l)
      {
        //gzwarn<<" parent link: "<<(*it_l)->GetName()<<std::endl;
        if(checked_links.find(*it_l) == checked_links.end()) {
          links_to_check.push(*it_l);
        }
      }

      // Add child links to the queue
      std::vector<gazebo::physics::LinkPtr> child_links = link->GetChildJointsLinks();
      for(std::vector<gazebo::physics::LinkPtr>::iterator it_l = child_links.begin();
          it_l != child_links.end();
          ++it_l)
      {
        //gzwarn<<" child link: "<<(*it_l)->GetName()<<std::endl;
        if(checked_links.find(*it_l) == checked_links.end()) {
          links_to_check.push(*it_l);
        }
      }
    }
  }

}
