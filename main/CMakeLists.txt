idf_component_register(SRCS "spi_sniffer_app.c"
                    INCLUDE_DIRS ".")
idf_build_set_property(COMPILE_DEFINITIONS "-DSPI_SNIFFER" APPEND)
