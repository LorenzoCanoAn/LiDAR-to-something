#ifndef VOXEL_VISUAL_H
#define VOXEL_VISUAL_H

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <voxelgrid_msgs/VoxelGridFloat32MultiarrayStamped.h>
#include <voxelgrid_msgs/VoxelGridInt16MultiarrayStamped.h>
namespace voxelgrid_rviz {

class VoxelGridVisual {
 public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  VoxelGridVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~VoxelGridVisual();

  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the Imu message.
  void setColor(float r, float g, float b, float a);

  void setBinaryDisplay(bool use_global_alpha);
  void setThreshold(float threshold);
  void setHidden(bool hidden);

  // Rerenders the point cloud from the list of points and UI-selected properties
  virtual void updatePointCloud() = 0;

  virtual void reset();

 protected:
  // The visible voxel grid ogre object
  boost::shared_ptr<rviz::PointCloud> voxel_grid_points_;

  // A SceneNode whose pose is set to match the coordinate frame of
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;

  // User settings
  float r_, g_, b_, a_;
  bool binary_display_;
  float threshold_;
  bool hidden_;
};


class Float32VoxelGridVisual : public VoxelGridVisual{
 public:
  using VoxelGridVisual::VoxelGridVisual;
  // Configure the visual to show the data in the message.
  void setMessage(const voxelgrid_msgs::VoxelGridFloat32MultiarrayStamped::ConstPtr& msg);

  void updatePointCloud() override;

  void reset() override;

 protected:
  // A local copy of the message is stored so that the voxelgrid can be
  // regenerated if the user changes the input
  voxelgrid_msgs::VoxelGridFloat32MultiarrayStamped latest_msg;
};

class Int16VoxelGridVisual : public VoxelGridVisual{
 public:
  using VoxelGridVisual::VoxelGridVisual;
  // Configure the visual to show the data in the message.
  void setMessage(const voxelgrid_msgs::VoxelGridInt16MultiarrayStamped::ConstPtr& msg);

  void updatePointCloud() override;

  void reset() override;

 protected:
  // A local copy of the message is stored so that the voxelgrid can be
  // regenerated if the user changes the input
  voxelgrid_msgs::VoxelGridInt16MultiarrayStamped latest_msg;
};

}  // end namespace voxelgrid_rviz

#endif  // VOXEL_VISUAL_H
