# Optional non-ROS original-core adapter, attached to the EXISTING shared ROS target.
option(XGC_WORLD_LIDAR_GPU "Compile explicit spherical-nearest GPU in the shared sensor" OFF)
if(XGC_WORLD_LIDAR_GPU)
  if(NOT TARGET shared_cloud_cpu_node)
    message(FATAL_ERROR "GPU needs the existing shared ROS entry and its catkin dependencies")
  endif()
  set(XGC_WORLD_LIDAR_GPU_SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/vendored/MARSIM"
    CACHE PATH "Fixed MARSIM memory entry (explicit override allowed)")
  if(NOT EXISTS "${XGC_WORLD_LIDAR_GPU_SOURCE_ROOT}/local_sensing/include/opengl_sim.hpp"
      OR NOT EXISTS "${XGC_WORLD_LIDAR_GPU_SOURCE_ROOT}/LICENSE")
    message(FATAL_ERROR "Explicit fixed MARSIM source and GPL license required")
  endif()
  set(_gpu_upstream "${XGC_WORLD_LIDAR_GPU_SOURCE_ROOT}/upstream/local_sensing/include")
  find_package(PCL REQUIRED COMPONENTS common io filters features search kdtree)
  find_package(OpenCV REQUIRED)
  find_package(OpenGL REQUIRED)
  find_package(glfw3 REQUIRED)
  find_package(OpenMP REQUIRED)
  find_path(XGC_WORLD_LIDAR_GPU_GLM_INCLUDE glm/glm.hpp)
  find_path(_gpu_khr_include KHR/khrplatform.h)
  if(NOT XGC_WORLD_LIDAR_GPU_GLM_INCLUDE OR NOT _gpu_khr_include)
    message(FATAL_ERROR "Actual GLM and Khronos headers required")
  endif()
  add_library(${PROJECT_NAME}_cloud_gpu STATIC src/shared_cloud_gpu.cpp
    "${_gpu_upstream}/glad.c" "${_gpu_upstream}/FOV_Checker/FOV_Checker.cpp")
  set_target_properties(${PROJECT_NAME}_cloud_gpu PROPERTIES POSITION_INDEPENDENT_CODE ON EXPORT_NAME CloudGpu)
  target_include_directories(${PROJECT_NAME}_cloud_gpu PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include> ${PCL_INCLUDE_DIRS}
    PRIVATE "${XGC_WORLD_LIDAR_GPU_SOURCE_ROOT}/local_sensing/include" "${_gpu_upstream}"
    "${XGC_WORLD_LIDAR_GPU_GLM_INCLUDE}" "${_gpu_khr_include}" ${OpenCV_INCLUDE_DIRS})
  target_compile_definitions(${PROJECT_NAME}_cloud_gpu PRIVATE XGC_WORLD_LIDAR_GPU=1
    ROOT_DIR="${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/marsim/")
  target_compile_options(${PROJECT_NAME}_cloud_gpu PRIVATE ${PCL_DEFINITIONS} ${OpenMP_CXX_FLAGS})
  # Keep the package's current CMake minimum; no new SDK/compiler framework.
  set_property(TARGET ${PROJECT_NAME}_cloud_gpu APPEND_STRING PROPERTY LINK_FLAGS " ${OpenMP_CXX_FLAGS}")
  target_link_libraries(${PROJECT_NAME}_cloud_gpu PUBLIC ${PROJECT_NAME}_cloud_cpu
    ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} glfw ${CMAKE_DL_LIBS})
  target_compile_definitions(${PROJECT_NAME}_sensor_system PRIVATE XGC_WORLD_LIDAR_GPU=1)
  target_link_libraries(${PROJECT_NAME}_sensor_system PUBLIC ${PROJECT_NAME}_cloud_gpu)
  install(TARGETS ${PROJECT_NAME}_cloud_gpu EXPORT XgcWorldLidarTargets
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} COMPONENT cloud_gpu)
  target_compile_definitions(shared_cloud_cpu_node PRIVATE XGC_WORLD_LIDAR_GPU=1)
  set_property(TARGET shared_cloud_cpu_node APPEND_STRING PROPERTY LINK_FLAGS " ${OpenMP_CXX_FLAGS}")
  target_link_libraries(shared_cloud_cpu_node PRIVATE ${PROJECT_NAME}_cloud_gpu)
  # Existing target/name/lifecycle remain; complete the optional dependency/assets install.
  install(FILES "${_gpu_upstream}/360camera.vs" "${_gpu_upstream}/camera.fs"
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/marsim/include COMPONENT cloud_gpu)
  install(DIRECTORY "${XGC_WORLD_LIDAR_GPU_SOURCE_ROOT}/"
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/marsim/source COMPONENT cloud_gpu
    PATTERN ".git" EXCLUDE PATTERN "build" EXCLUDE)
  install(FILES package.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} COMPONENT cloud_gpu)
  install(FILES config/gpu_spherical_nearest.yaml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config COMPONENT cloud_gpu)
endif()
