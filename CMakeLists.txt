# Cleaned 20210319
idf_component_register(
	SRCS "commands.c"
	INCLUDE_DIRS "."
	REQUIRES common
	PRIV_REQUIRES onewire pca9555 sitewhere ssd1306 thingsboard telnet_server
)
