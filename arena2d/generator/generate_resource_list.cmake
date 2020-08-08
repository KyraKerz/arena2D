add_executable(resourcegenerator generator/resourcelistgenerator.cpp)

set(RESOURCE_FILES
	${PROJECT_SOURCE_DIR}/generated/resources/resource_list.cpp
	${PROJECT_SOURCE_DIR}/generated/resources/resource_list.h)

add_custom_command(OUTPUT ${RESOURCE_FILES}
					COMMAND resourcegenerator ${PROJECT_SOURCE_DIR}/data/resource_list.res
												resource_list.h resource_list.cpp
					DEPENDS ${PROJECT_SOURCE_DIR}/data/resource_list.res resourcegenerator
					WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/generated/resources
					)
