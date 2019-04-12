set(COMPONENT_SRCS "src/i2c.c"
				   "src/ssd1306_i2c.c"
				   "src/fonts.c"
				   "src/font_glcd_5x7.c"
				   "src/font_tahoma_8pt")
set(COMPONENT_ADD_INCLUDEDIRS "include")

set(COMPONENT_REQUIRES "")

register_component()
