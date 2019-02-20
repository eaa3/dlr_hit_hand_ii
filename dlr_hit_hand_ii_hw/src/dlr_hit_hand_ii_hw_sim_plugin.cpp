
#include <dlr_hit_hand_ii_hw/dlr_hit_hand_ii_hw_sim_plugin.h>

namespace dlr_hit_hand_ii_hw
{


// Overloaded Gazebo entry point
void DLRHitHandIIHWSimPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  GazeboRosControlPlugin::Load(parent, sdf);
}



// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DLRHitHandIIHWSimPlugin);

} // namespace
