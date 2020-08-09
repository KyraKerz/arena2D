cmake_minimum_required(VERSION 2.8)
project(arena2d)

# additional compile options (warnings)
add_compile_options(-Wreturn-type -Wuninitialized -g)

# adding compile options
set(NO_ROS FALSE CACHE BOOL "if set to true arena is compiled without ros")
# add_compile_options("-DARENA_USE_ROS")# TODO
set(NO_PYTHON TRUE CACHE BOOL "if set to true arena is compiled with the python interpreter")

if(NO_ROS)
	message(STATUS "ROS disabled")
else()
	message(STATUS "ROS enabled")
endif()



if(NO_PYTHON)
	message(STATUS "Python disabled")
else()
	message(STATUS "Python enabled")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
if(NOT NO_ROS)
find_package(catkin REQUIRED COMPONENTS
			roscpp
			roslib
			rospy
			std_msgs
			sensor_msgs
			message_generation
			rostest
)
endif()

if(POLICY CMP0004 AND ${CMAKE_VERSION} VERSION_LESS "3.16.2")
	cmake_policy(SET CMP0004 OLD) # use legacy OpenGL drivers
endif()
# find packages
if(POLICY CMP0072)
	cmake_policy(SET CMP0072 OLD) # use legacy OpenGL drivers
endif()
find_package(SDL2 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Freetype REQUIRED)

# find right python version
if(NOT NO_PYTHON)
	# find_package(PythonLibs REQUIRED)
	# use the env in anaconda
	set(PYTHONLIBS_VERSION_STRING "3.7")
	set(PYTHON_INCLUDE_DIRS "$ENV{HOME}/anaconda3/include/python3.7m")
	set(PYTHON_LIBRARIES "$ENV{HOME}/anaconda3/lib/libpython3.7m.so")
endif()

include(user_config.cmake OPTIONAL)

if(NOT NO_PYTHON)
	message(STATUS "Using python version ${PYTHONLIBS_VERSION_STRING}")
endif()

# show found libraries and include directories
set(LIBRARY_INFO_MESSAGE 	"Libraries:\n"
							"  -> sdl2:     ${SDL2_LIBRARIES}\n"
							"  -> opengl:   ${OPENGL_LIBRARIES}\n"
							"  -> freetype: ${FREETYPE_LIBRARIES}\n")
set(INCLUDE_INFO_MESSAGE	"Include directories:\n"
							"  -> sdl2:     ${SDL2_INCLUDE_DIRS}\n"
							"  -> opengl:   ${OPENGL_INCLUDE_DIRS}\n"
							"  -> freetype: ${FREETYPE_INCLUDE_DIRS}\n")
if(NOT NO_PYTHON)
	set(INCLUDE_INFO_MESSAGE "${INCLUDE_INFO_MESSAGE}  -> python:   ${PYTHON_INCLUDE_DIRS}\n")
	set(LIBRARY_INFO_MESSAGE "${LIBRARY_INFO_MESSAGE}  -> python:   ${PYTHON_LIBRARIES}\n")
endif()
 
message(${INCLUDE_INFO_MESSAGE})
message(${LIBRARY_INFO_MESSAGE})

# config global setting files
if(NO_ROS)
	set(PACKAGE_PATH ".") # if ros is not enabled, set 
else()
	set(PACKAGE_PATH ${PROJECT_SOURCE_DIR})
endif()
configure_file(engine/globalSettings.cpp.in ${PROJECT_SOURCE_DIR}/engine/globalSettings.cpp)

# include external cmake files for generators
include(generator/generate_shader.cmake)
include(generator/generate_resource_list.cmake)
include(generator/generate_settings.cmake)

# include source files listed in external cmake files
include(sources.cmake)

# adding box2d
add_subdirectory(box2d/src/)

## Generate messages in the 'msg' folder
if(NOT NO_ROS)
add_message_files(
	FILES
	observation.msg
)
endif()

## Generate services in the 'srv' folder
if(NOT NO_ROS)
add_service_files(
	FILES
	InteractionDiscActs.srv
	arenaCommand.srv
)
endif()

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
if(NOT NO_ROS)
generate_messages(
	DEPENDENCIES
	sensor_msgs
	geometry_msgs
	std_msgs
)
endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arena2d
 CATKIN_DEPENDS
	message_runtime # necessary for using package defined msgs and srvs.
)

###########
## Build ##
###########

# include directories
include_directories(
	./
	./box2d/include/
	${OPENGL_INCLUDE_DIRS}
	${SDL2_INCLUDE_DIRS}
	${FREETYPE_INCLUDE_DIRS}
	# /home/joe/projects/rl/devel/include #TODO need to delete later
)
if(NOT NO_ROS)
	include_directories(${catkin_INCLUDE_DIRS})
	message("added catkin_include_dirs: ${catkin_INCLUDE_DIRS}")
endif()
if(NOT NO_PYTHON)
	include_directories(${PYTHON_INCLUDE_DIRS})
endif()

# creating folders to put generated stuff in
add_custom_target(CREATE_GEN_DIRECTORIES ${CMAKE_COMMAND} -E make_directory
										generated/shader/
										generated/resources/
										generated/settings/
										WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

# set preprocessor defines for arena based on configuration
if(NOT NO_ROS)
	add_compile_options("-DARENA_USE_ROS")
	message(STATUS "Compiling with define ARENA_USE_ROS")
endif()
if(NOT NO_PYTHON)
	add_compile_options("-DARENA_USE_PYTHON")
	message(STATUS "Compiling with define ARENA_USE_PYTHON")
	if(${PYTHONLIBS_VERSION_STRING} VERSION_GREATER "2.9")
		message(STATUS "Compiling with define ARENA_PYTHON_VERSION_3")
		add_compile_options("-DARENA_PYTHON_VERSION_3")
	endif()
endif()

# adding arena executable
add_executable(arena ${SHADER_FILES} ${RESOURCE_FILES} ${SETTINGS_FILES} ${ARENA_SOURCES})

# adding dependencies
add_dependencies(arena CREATE_GEN_DIRECTORIES)
if(NOT NO_ROS)
	add_dependencies(arena ${PROJECT_NAME}_generate_messages_cpp)
	# add_dependencies(arena ${${PROJECT_NAME}_EXPORTED_TARGETS})
endif()
# add_custom_command(TARGET arena POST_BUILD
#                    COMMAND ${CMAKE_COMMAND} -E copy_directory
# 							  ${CMAKE_SOURCE_DIR}/data $<TARGET_FILE_DIR:arena>)

# add needed fonts to the folder have executation
# add_custom_command(TARGET arena POST_BUILD
#                    COMMAND ${CMAKE_COMMAND} -E copy_directory
# 				   ${PROJECT_SOURCE_DIR}/data $<TARGET_FILE_DIR:arena>/data
# 				   COMMAND ${CMAKE_COMMAND} -E copy_directory
# 				   ${PROJECT_SOURCE_DIR}/level $<TARGET_FILE_DIR:arena>/level
# 				   COMMAND ${CMAKE_COMMAND} -E copy
# 				   ${PROJECT_SOURCE_DIR}/settings.st $<TARGET_FILE_DIR:arena>/settings.st)
# specifying link libraries
set(ARENA_LINK_LIBRARIES
	box2d
	${OPENGL_LIBRARIES}
	${SDL2_LIBRARIES}
	${FREETYPE_LIBRARIES}
)
if(NOT NO_ROS)
	set(ARENA_LINK_LIBRARIES ${ARENA_LINK_LIBRARIES} ${catkin_LIBRARIES})
endif()
if(NOT NO_PYTHON)
	set(ARENA_LINK_LIBRARIES ${ARENA_LINK_LIBRARIES} ${PYTHON_LIBRARIES})
endif()

target_link_libraries(arena "${ARENA_LINK_LIBRARIES}")
set_property(TARGET arena PROPERTY CXX_STANDARD 11)

# adding remote commandline executable
if(NOT NO_ROS)
	#adding remote_commandline target
	add_executable(remote_commandline remote_commandline/remote_commandline.cpp arena/Command.h)
	add_dependencies(remote_commandline arena2d_generate_messages_cpp)
	target_link_libraries(remote_commandline ${catkin_LIBRARIES})
	set_property(TARGET remote_commandline PROPERTY CXX_STANDARD 11)

endif()
if(CATKIN_ENABLE_TESTING)
	# put our python test scripts into destionation folder
	# so that it can be invoked in the launch file or test file directly with its name
	# dont forget to change the mod of corresponding python scripts, otherwise the test will failed!
	add_rostest(launch/interactionServ.test)
endif()
