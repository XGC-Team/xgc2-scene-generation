# Shared native GPU library is supplied by libxgc2-sensing-gpu.
option(XGC_WORLD_LIDAR_GPU "Compile the shared GPU sensing client" ON)
if(XGC_WORLD_LIDAR_GPU)
  find_path(XGC2_SENSING_INCLUDE xgc2_sensing/gpu.h REQUIRED)
  add_library(${PROJECT_NAME}_cloud_gpu STATIC src/shared_cloud_gpu.cpp src/sensor_metadata.cpp)
  target_include_directories(${PROJECT_NAME}_cloud_gpu PUBLIC include ${PCL_INCLUDE_DIRS} ${XGC2_SENSING_INCLUDE})
  target_link_libraries(${PROJECT_NAME}_cloud_gpu PUBLIC ${PCL_LIBRARIES} ${CMAKE_DL_LIBS})

endif()
