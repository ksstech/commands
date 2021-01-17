idf_component_register(
	SRCS "commands.c"
	INCLUDE_DIRS "."
	PRIV_REQUIRES hal_esp32 http identity m90e26 mqtt onewire pca9555 printfx rules sitewhere ssd1306 stringsX systiming telnet_server thingsboard
)
