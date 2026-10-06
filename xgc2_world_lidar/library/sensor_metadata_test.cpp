#include <xgc2_world_lidar/sensor_metadata.hpp>
#include <cassert>
#include <stdexcept>
using namespace xgc2_world_lidar;
int main(){SensorMetadata m;m.backend="gpu";m.observation_model="lidar_scan";m.range_m=5;m.min_range_m=.1;m.point_cover_spacing_m=.1;m.publish_rate_hz=12;m.h_fov_deg=120;m.v_fov_deg=60;m.h_res=240;m.v_res=120;m.frame_id="world";m.stamp_policy="pose";m.pose_type="geometry_msgs/PoseStamped";validateGpuSensorMetadata(m);
for(int test=0;test<5;++test){auto bad=m;if(test==0)bad.backend="cpu";if(test==1)bad.h_res=0;if(test==2)bad.range_m=bad.min_range_m;if(test==3)bad.point_cover_spacing_m=1;if(test==4)bad.v_res=30;bool failed=false;try{validateGpuSensorMetadata(bad);}catch(const std::invalid_argument&){failed=true;}assert(failed);}return 0;}
