# COMMANDS
set( srcs "commands.c" )
set( include_dirs "." )
set( requires common )
set( priv_requires actuators hal_esp32 http identity m90e26 mcp342x mqtt onewire printfx pca9555
	rtos-support rules ssd1306 sitewhere stringsX syslog systiming telnet_server thingsboard )
idf_component_register(SRCS ${srcs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires} PRIV_REQUIRES ${priv_requires})
