# COMMANDS

set( srcs "commands.c" )
set( include_dirs "." )
set( priv_include_dirs )
idf_build_get_property( EXTRA_REQUIRES EXTRA_REQUIRES )
set( requires "main" ) #
set( priv_requires "actuators modbus mqtt" )

idf_component_register(
	SRCS ${srcs}
	INCLUDE_DIRS ${include_dirs}
	PRIV_INCLUDE_DIRS ${priv_include_dirs}
	REQUIRES ${requires}
	PRIV_REQUIRES ${priv_requires}
)
