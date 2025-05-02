cmake_minimum_required(VERSION 3.5)
project(photo)

# Dependencias de ROS 2 y otros paquetes
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Incluir directorios de encabezados
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Archivos fuente
set(SOURCES
  src/photo/image_saver.cpp
  src/main.cpp
)

# Crear el ejecutable
add_executable(image_saver ${SOURCES})

# Vincular librerías necesarias
target_link_libraries(image_saver
  ${OpenCV_LIBS}
  cv_bridge::cv_bridge
)

# Especificar dependencias del ejecutable
ament_target_dependencies(image_saver
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# Instalar ejecutable y directorios
install(TARGETS
  image_saver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Exportar información del paquete
ament_package()
