add_executable(shadergenerator generator/shadergenerator.c)

# generating shader files
set(SHADER_NAMES
	color2d
	colorplex2d
	sprite
	text
	particle
)
FOREACH(N ${SHADER_NAMES})
	set(SHADER_FILES ${SHADER_FILES} "generated/shader/${N}Shader.h")
	add_custom_command(	OUTPUT ${PROJECT_SOURCE_DIR}/generated/shader/${N}Shader.h 
						COMMAND shadergenerator shader/${N}Shader_vs.glsl
												shader/${N}Shader_fs.glsl
												generated/shader/${N}Shader.h
						DEPENDS shader/${N}Shader_vs.glsl shader/${N}Shader_fs.glsl
								shadergenerator
						WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
					)
ENDFOREACH(N)
