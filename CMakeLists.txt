# COMMANDS

set( srcs "commands.c" )
set( include_dirs "." )
set( priv_include_dirs )
set( requires common )
set( priv_requires 
	actuators hal_esp32 http identity lis2hh12 ltr329als m90e26 mcp342x modbus mpl3115 mqtt onewire 
	printfx pca9555 pycoproc rtos-support rules ssd1306 si70xx stringsX syslog systiming telnet_server  
)

list( APPEND priv_requires sitewhere thingsboard )

idf_component_register(
	SRCS ${srcs} 
	INCLUDE_DIRS ${include_dirs} 
	PRIV_INCLUDE_DIRS ${priv_include_dirs} 
	REQUIRES ${requires} 
	PRIV_REQUIRES ${priv_requires}
)
