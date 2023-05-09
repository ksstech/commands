# COMMANDS

set( srcs "commands.c" )
set( include_dirs "." )
set( priv_include_dirs )
set( requires )
set( priv_requires 
	main
	common hal_esp32
	lis2hh12 ltr329als m90e26 mcp342x modbus mpl3115 mqtt onewire pca9555 pcf8574 pycoproc ssd1306 si70xx 
	actuators http identity printfx rtos-support rules stringsX syslog systiming telnet_server
)

idf_component_register(
	SRCS ${srcs} 
	INCLUDE_DIRS ${include_dirs} 
	PRIV_INCLUDE_DIRS ${priv_include_dirs} 
	REQUIRES ${requires} 
	PRIV_REQUIRES ${priv_requires}
)
