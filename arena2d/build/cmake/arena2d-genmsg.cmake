# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "arena2d: 1 messages, 2 services")

set(MSG_I_FLAGS "-Iarena2d:/home/joe/projects/rl/src/arena2d/msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(arena2d_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/joe/projects/rl/src/arena2d/srv/interactionDiscActs.srv" NAME_WE)
add_custom_target(_arena2d_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arena2d" "/home/joe/projects/rl/src/arena2d/srv/interactionDiscActs.srv" "geometry_msgs/Vector3:sensor_msgs/LaserScan:arena2d/observation:std_msgs/Header"
)

get_filename_component(_filename "/home/joe/projects/rl/src/arena2d/srv/arenaCommand.srv" NAME_WE)
add_custom_target(_arena2d_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arena2d" "/home/joe/projects/rl/src/arena2d/srv/arenaCommand.srv" ""
)

get_filename_component(_filename "/home/joe/projects/rl/src/arena2d/msg/observation.msg" NAME_WE)
add_custom_target(_arena2d_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arena2d" "/home/joe/projects/rl/src/arena2d/msg/observation.msg" "geometry_msgs/Vector3:sensor_msgs/LaserScan:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(arena2d
  "/home/joe/projects/rl/src/arena2d/msg/observation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/LaserScan.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arena2d
)

### Generating Services
_generate_srv_cpp(arena2d
  "/home/joe/projects/rl/src/arena2d/srv/interactionDiscActs.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/LaserScan.msg;/home/joe/projects/rl/src/arena2d/msg/observation.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arena2d
)
_generate_srv_cpp(arena2d
  "/home/joe/projects/rl/src/arena2d/srv/arenaCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arena2d
)

### Generating Module File
_generate_module_cpp(arena2d
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arena2d
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(arena2d_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(arena2d_generate_messages arena2d_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joe/projects/rl/src/arena2d/srv/interactionDiscActs.srv" NAME_WE)
add_dependencies(arena2d_generate_messages_cpp _arena2d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joe/projects/rl/src/arena2d/srv/arenaCommand.srv" NAME_WE)
add_dependencies(arena2d_generate_messages_cpp _arena2d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joe/projects/rl/src/arena2d/msg/observation.msg" NAME_WE)
add_dependencies(arena2d_generate_messages_cpp _arena2d_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arena2d_gencpp)
add_dependencies(arena2d_gencpp arena2d_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arena2d_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(arena2d
  "/home/joe/projects/rl/src/arena2d/msg/observation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/LaserScan.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arena2d
)

### Generating Services
_generate_srv_eus(arena2d
  "/home/joe/projects/rl/src/arena2d/srv/interactionDiscActs.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/LaserScan.msg;/home/joe/projects/rl/src/arena2d/msg/observation.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arena2d
)
_generate_srv_eus(arena2d
  "/home/joe/projects/rl/src/arena2d/srv/arenaCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arena2d
)

### Generating Module File
_generate_module_eus(arena2d
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arena2d
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(arena2d_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(arena2d_generate_messages arena2d_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joe/projects/rl/src/arena2d/srv/interactionDiscActs.srv" NAME_WE)
add_dependencies(arena2d_generate_messages_eus _arena2d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joe/projects/rl/src/arena2d/srv/arenaCommand.srv" NAME_WE)
add_dependencies(arena2d_generate_messages_eus _arena2d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joe/projects/rl/src/arena2d/msg/observation.msg" NAME_WE)
add_dependencies(arena2d_generate_messages_eus _arena2d_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arena2d_geneus)
add_dependencies(arena2d_geneus arena2d_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arena2d_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(arena2d
  "/home/joe/projects/rl/src/arena2d/msg/observation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/LaserScan.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arena2d
)

### Generating Services
_generate_srv_lisp(arena2d
  "/home/joe/projects/rl/src/arena2d/srv/interactionDiscActs.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/LaserScan.msg;/home/joe/projects/rl/src/arena2d/msg/observation.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arena2d
)
_generate_srv_lisp(arena2d
  "/home/joe/projects/rl/src/arena2d/srv/arenaCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arena2d
)

### Generating Module File
_generate_module_lisp(arena2d
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arena2d
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(arena2d_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(arena2d_generate_messages arena2d_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joe/projects/rl/src/arena2d/srv/interactionDiscActs.srv" NAME_WE)
add_dependencies(arena2d_generate_messages_lisp _arena2d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joe/projects/rl/src/arena2d/srv/arenaCommand.srv" NAME_WE)
add_dependencies(arena2d_generate_messages_lisp _arena2d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joe/projects/rl/src/arena2d/msg/observation.msg" NAME_WE)
add_dependencies(arena2d_generate_messages_lisp _arena2d_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arena2d_genlisp)
add_dependencies(arena2d_genlisp arena2d_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arena2d_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(arena2d
  "/home/joe/projects/rl/src/arena2d/msg/observation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/LaserScan.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arena2d
)

### Generating Services
_generate_srv_nodejs(arena2d
  "/home/joe/projects/rl/src/arena2d/srv/interactionDiscActs.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/LaserScan.msg;/home/joe/projects/rl/src/arena2d/msg/observation.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arena2d
)
_generate_srv_nodejs(arena2d
  "/home/joe/projects/rl/src/arena2d/srv/arenaCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arena2d
)

### Generating Module File
_generate_module_nodejs(arena2d
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arena2d
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(arena2d_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(arena2d_generate_messages arena2d_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joe/projects/rl/src/arena2d/srv/interactionDiscActs.srv" NAME_WE)
add_dependencies(arena2d_generate_messages_nodejs _arena2d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joe/projects/rl/src/arena2d/srv/arenaCommand.srv" NAME_WE)
add_dependencies(arena2d_generate_messages_nodejs _arena2d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joe/projects/rl/src/arena2d/msg/observation.msg" NAME_WE)
add_dependencies(arena2d_generate_messages_nodejs _arena2d_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arena2d_gennodejs)
add_dependencies(arena2d_gennodejs arena2d_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arena2d_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(arena2d
  "/home/joe/projects/rl/src/arena2d/msg/observation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/LaserScan.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arena2d
)

### Generating Services
_generate_srv_py(arena2d
  "/home/joe/projects/rl/src/arena2d/srv/interactionDiscActs.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/LaserScan.msg;/home/joe/projects/rl/src/arena2d/msg/observation.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arena2d
)
_generate_srv_py(arena2d
  "/home/joe/projects/rl/src/arena2d/srv/arenaCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arena2d
)

### Generating Module File
_generate_module_py(arena2d
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arena2d
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(arena2d_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(arena2d_generate_messages arena2d_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joe/projects/rl/src/arena2d/srv/interactionDiscActs.srv" NAME_WE)
add_dependencies(arena2d_generate_messages_py _arena2d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joe/projects/rl/src/arena2d/srv/arenaCommand.srv" NAME_WE)
add_dependencies(arena2d_generate_messages_py _arena2d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joe/projects/rl/src/arena2d/msg/observation.msg" NAME_WE)
add_dependencies(arena2d_generate_messages_py _arena2d_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arena2d_genpy)
add_dependencies(arena2d_genpy arena2d_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arena2d_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arena2d)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arena2d
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(arena2d_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(arena2d_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(arena2d_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arena2d)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arena2d
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(arena2d_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(arena2d_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(arena2d_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arena2d)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arena2d
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(arena2d_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(arena2d_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(arena2d_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arena2d)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arena2d
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(arena2d_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(arena2d_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(arena2d_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arena2d)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arena2d\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arena2d
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(arena2d_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(arena2d_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(arena2d_generate_messages_py std_msgs_generate_messages_py)
endif()
