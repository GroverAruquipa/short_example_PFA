cmake_minimum_required(VERSION 3.8.0)
project(argos3_examples)

# Estándar C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Incluimos los headers básicos necesarios
include_directories(/home/groverpc/argos3/src /usr/include/lua5.2)

# Solo entramos en la carpeta del ejemplo que necesitas (por ejemplo, footbot_diffusion)
# Si Giovani te pidió otro, cambia 'footbot_diffusion' por el nombre de la carpeta correspondiente
add_subdirectory(controllers/footbot_diffusion)
