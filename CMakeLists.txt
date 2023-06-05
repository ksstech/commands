# COMMANDS

set( srcs "commands.c" )
set( include_dirs "." )
set( priv_include_dirs )
set( requires )
idf_build_get_property( cmakeDEVICES cmakeDEVICES )
set( priv_requires common hal_esp32 modbus mqtt actuators http identity printfx rtos-support rules stringsX syslog systiming telnet_server ${cmakeDEVICES} )

idf_component_register(
	SRCS ${srcs} 
	INCLUDE_DIRS ${include_dirs} 
	PRIV_INCLUDE_DIRS ${priv_include_dirs} 
	REQUIRES ${requires} 
	PRIV_REQUIRES ${priv_requires}
)
