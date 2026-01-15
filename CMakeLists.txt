cmake_minimum_required(VERSION 3.8)

# ROS 2 uses ament instead of catkin
find_package(ament_cmake REQUIRED)

# Install your resource folders into the package share
install(
  DIRECTORY
    launch
    meshes
    urdf
  DESTINATION share/${PROJECT_NAME}
)

# Finalize the ament package
ament_package()
