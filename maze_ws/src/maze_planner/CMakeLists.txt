cmake_minimum_required(VERSION 3.8)
project(maze_planner)

# Padrão do ROS2: usar C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 1) Dependências
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

# 2) Executáveis

# Parte 1: planner offline (já funcionando)
add_executable(rota-otimizada-parte1 src/rota-otimizada-parte1.cpp)
ament_target_dependencies(rota-otimizada-parte1
  rclcpp
  cg_interfaces
)

# Parte 2: mapper online
add_executable(mapeamento-parte2 src/mapeamento-parte2.cpp)
ament_target_dependencies(mapeamento-parte2
  rclcpp
  cg_interfaces
)

# 3) Instalar executáveis (pro ros2 run achar)
install(TARGETS
  rota-otimizada-parte1
  mapeamento-parte2
  DESTINATION lib/${PROJECT_NAME}
)

# 4) Finalizar pacote
ament_package()
