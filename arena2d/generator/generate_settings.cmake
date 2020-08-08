add_executable(settingsgenerator generator/settingsgenerator.c generator/list.c)

set(SETTINGS_FILES
	${PROJECT_SOURCE_DIR}/generated/settings/dynamicSettings.cpp)

add_custom_command(OUTPUT ${SETTINGS_FILES}
					COMMAND settingsgenerator ${PROJECT_SOURCE_DIR}/engine/settingsStructs.h dynamicSettings.cpp
					DEPENDS ${PROJECT_SOURCE_DIR}/engine/settingsStructs.h generator/settingsgenerator.c
					WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/generated/settings
					)
