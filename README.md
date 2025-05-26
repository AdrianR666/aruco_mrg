# ArUco Detector ROS 2

Este paquete detecta marcadores ArUco usando OpenCV, estima su pose con respecto a una cámara y publica las transformaciones (`tf2`) correspondientes. También publica una imagen con los marcadores detectados y sus ejes.

## 🧠 Características

- Detección de marcadores ArUco con OpenCV
- Estimación de pose y publicación como `tf`
- Visualización en RViz2 y mediante `rqt_image_view`

## 📦 Requisitos

- ROS 2 Humble (u otra versión compatible)
- OpenCV (incluyendo `aruco`)
- `cv_bridge`, `image_transport`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`

> ⚠️ Asegúrate de tener tu cámara calibrada y de haber generado la `camera_matrix` y los `dist_coeffs`.


---

## 🚀 Instalación

# Clonar el repositorio de utilidades (aruco_mrg)
git clone https://github.com/AdrianR666/aruco_mrg.git

# Instalar dependencias si no están
sudo apt update
sudo apt install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-tf2* libopencv-dev

# Compilar
- cd ~/aruco_ws
- colcon build
- source install/setup.bash


