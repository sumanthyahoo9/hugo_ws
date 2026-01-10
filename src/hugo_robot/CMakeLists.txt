cmake_minimum_required(VERSION 3.8)
project(hugo_robot)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install URDF files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Add after the existing install() commands
install(PROGRAMS
  scripts/trajectory_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

# Add the controller
install(PROGRAMS
  scripts/trajectory_recorder.py
  scripts/simple_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Docking sequence
install(PROGRAMS
  scripts/trajectory_recorder.py
  scripts/simple_controller.py
  scripts/docking_sequence.py
  DESTINATION lib/${PROJECT_NAME}
)

# Workspace analyzer
install(PROGRAMS
  scripts/trajectory_recorder.py
  scripts/simple_controller.py
  scripts/docking_sequence.py
  scripts/workspace_analyzer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()