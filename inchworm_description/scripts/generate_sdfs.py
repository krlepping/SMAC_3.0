#!/usr/bin/env python3

import glob, subprocess, rospkg, os

import xml.etree.ElementTree as ET

rospack = None

def augmentSDF(sdf, model_name, params="x y z rx ry rz fixed idx"):
  '''
  Takes in an SDF, and wraps everything in <sdf></sdf> in a xacro:macro. Also sets the parameter list for the macro,
      and paramaterizes model and link names to be based on the idx parameter.
  '''

  tree = ET.fromstring(sdf)

  # Convert top level tag into a model tag
  model = list(tree)[0]
  model.tag = "model"
  model.set("name", model_name + "_${idx}")

  # Convert stupid link names to normal link names
  #   (the stupid names are so that RViz grabs the right transforms to visualize)
  for link in model.findall("link"):
    link_name = link.get("name")
    link.set("name", link_name + "_${idx}")

  macro = ET.Element("xacro:macro")
  macro.set("name", model_name)
  macro.set("params", params)
  macro.insert(0, model)

  # Create a pose that depends on these parameters
  pose = ET.Element("pose")
  pose.text = "${x} ${y} ${z} ${rx} ${ry} ${rz}"

  xacro_if = ET.Element("xacro:if")
  xacro_if.set("value", "${fixed}")

  fixed_joint = ET.Element("joint")
  fixed_joint.set("name", "fixed_to_world")
  fixed_joint.set("type", "fixed")

  parent = ET.Element("parent")
  parent.text = "world"
  child = ET.Element("child")
  child.text = model.findall("link")[0].get("name").split("/")[0]

  # fixed_joint.insert(0, pose)
  fixed_joint.insert(0, child)
  fixed_joint.insert(0, parent)

  xacro_if.insert(0, fixed_joint)
  
  model.insert(0, pose)
  model.insert(0, xacro_if)

  # Create a new root that the model is a child of
  new_root = ET.Element('sdf')
  new_root.set("version", "1.7")
  new_root.set("xmlns:xacro", "http://www.ros.org/wiki/xacro")

  new_root.insert(0, macro)

  # return ET.tostring(new_root).decode("utf-8")
  return new_root

def generateModelSpawner(macro_names):
  # Parent SDF tag
  sdf = ET.Element("sdf")
  sdf.set("version", "1.7")
  sdf.set("xmlns:xacro", "http://www.ros.org/wiki/xacro")

  # Create and insert magnet include
  magnet_include = ET.Element("xacro:include")
  magnet_include.set("filename", "$(find inchworm_description)/urdf/inchworm_magnets.sdf")
  sdf.insert(0, magnet_include)

  # The master model that everything is loaded under. <name>_soup is a convention from lcsr_assembly. Name overwritten by launch file.
  model = ET.Element("model")
  model.set("name", "inchworm_soup")

  # Create magnet plugin
  plugin = ET.Element("plugin")
  plugin.set("name", "assembly_soup")
  plugin.set("filename", "libassembly_soup_plugin.so")

  # Magnet sim reference frame for published transforms
  world_frame = ET.Element("tf_world_frame")
  world_frame.text = "world"
  plugin.insert(0, world_frame)

  # Whether to publish the mate list
  publish_mates = ET.Element("publish_active_mates")
  publish_mates.text = "1"
  plugin.insert(1, publish_mates)

  # Add the mate models and atoms to the plugin
  plugin.insert(2, ET.Element("xacro:inchworm_mate"))
  plugin.insert(3, ET.Element("xacro:iw_foot_atom"))
  plugin.insert(4, ET.Element("xacro:shingle_atom"))

  # Insert the plugin parameters into the soup model
  model.insert(0, plugin)

  for name in macro_names:
    include = ET.Element("xacro:include")

    include.set("filename", f"$(find inchworm_description)/sdf/{name}.sdf")

    macro = ET.Element("xacro:" + name)

    for param in "x y z rx ry rz fixed idx".split(" "):
      macro.set(param, "0")

    model.append(macro)
    sdf.insert(0, include)

  sdf.append(model)

  return ET.tostring(sdf).decode("utf-8")

def generateShingle(sdf):
  shingle = augmentSDF(sdf, "shingle_description")

  return ET.tostring(shingle).decode("utf-8")

def generateInchworm(sdf):
  inchworm = augmentSDF(sdf, "inchworm_description")

  # Add `_${idx}`` to pose relative_to
  macro = inchworm.find("xacro:macro")
  model = macro.find("model")
  links = model.findall("link")
  joints = model.findall("joint")

  print("Inchworm links:")
  for link in links:
    print(link.get("name"))

  # Link poses point to joints, which aren't currently idx parameterized
  # for link in links:
  #   pose = link.find("pose")

  #   # Hooray short circuiting!
  #   if pose is not None and pose.get("relative_to") is not None:
  #     pose.set("relative_to", pose.get("relative_to") + "_${idx}")

  for joint in joints:
    pose = joint.find("pose")

    # Hooray short circuiting!
    if pose is not None and pose.get("relative_to") is not None:
      pose.set("relative_to", pose.get("relative_to") + "_${idx}")

    parent = joint.find("parent")
    parent.text += "_${idx}"

    child = joint.find("child")
    child.text += "_${idx}"

  return ET.tostring(inchworm).decode("utf-8")

def generateRoof(sdf):
  roof = augmentSDF(sdf, "roof_description", params="x y z rx ry rz fixed idx width height")

  macro = roof.find("xacro:macro")
  model = macro.find("model")
  link = model.find("link")

  visual = link.find("visual")
  collision = link.find("collision")

  cur_pose = visual.find("pose").text.split(" ")
  cur_pose[0] = "${width/2}"
  cur_pose[1] = "${height/2}"

  visual.find("pose").text = ' '.join(cur_pose)
  collision.find("pose").text = ' '.join(cur_pose)

  vis_size = visual.find("geometry").find("box").find("size")
  col_size = collision.find("geometry").find("box").find("size")
  
  cur_size = vis_size.text.split(" ")
  cur_size[0] = "${width}"
  cur_size[1] = "${height}"

  vis_size.text = ' '.join(cur_size)
  col_size.text = ' '.join(cur_size)

  return ET.tostring(roof).decode("utf-8")

def main():
  '''
  The purpose of this script is to generate an SDF for every URDF in the /urdf directory of inchworm_description. RViz needs URDFs to visualize properly,
  but the magnet sim requires SDFs to function properly*. Therefore we need a copy of each. URDF->SDF conversion is lossless, so we can automate the generation
  with this script using gazebo command line tools. This script wraps the generated <model> tag in a xacro macro with the filename, so that it can be invoked
  by a "master" SDF that will act as the parent model for all sub models. The `all_models.sdf` file is also generated by this script.
  '''

  rospack = rospkg.RosPack()

  # Path to the inchworm_description package
  desc_loc = rospack.get_path("inchworm_description")
  
  # Scan and grab every file that ends in .urdf in /urdf
  urdf_files = glob.glob(f"{desc_loc}/urdf/*.urdf")

  # For each URDF
  for urdf in urdf_files:
    sdf_filename = urdf.split("/")[-1][:-4] + "sdf"

    sdf_path = f"{desc_loc}/sdf/{sdf_filename}"

    # Generate an SDF with the gazebo command line interface
    proc = subprocess.Popen(["gz", "sdf", "-p", urdf], stdout=subprocess.PIPE)
    sdf_data = proc.stdout.read().decode("utf-8")

    NAME_FN_MAP = {
      "inchworm_description": generateInchworm,
      "shingle": generateShingle,
      "roof": generateRoof
    }

    # Convert into an XML object
    # Wrap the <model> tag with a <xacro:macro> tag
    name = sdf_filename[:-4]
    if name in NAME_FN_MAP:
      new_sdf = NAME_FN_MAP[name](sdf_data)
    else:
      print(f"Skipping unmapped urdf: {name}.urdf")

    # Write the contents to `sdf/<same_name>.sdf`
    with open(sdf_path, 'w+') as f:
      f.write(new_sdf)

  # Reset all_models.sdf file
  #if os.path.exists(f"{desc_loc}/sdf/all_models.sdf"):
  #  os.remove(f"{desc_loc}/sdf/all_models.sdf")

  sdf_files = glob.glob(f"{desc_loc}/sdf/*.sdf")
  macro_names = [f.split("/")[-1][:-4] for f in sdf_files]

  print(macro_names)

  all_sdfs = generateModelSpawner(macro_names)

  #with open(f"{desc_loc}/sdf/all_models.sdf", "w+") as f:
    # Don't do this right now, all_models is heavily modified.
    # f.write(all_sdfs)

  #  pass

if __name__ == "__main__":
  main()