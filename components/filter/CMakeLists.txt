idf_component_register(
    SRCS "filter.c"
    INCLUDE_DIRS "."  # dove c'è piston.h
    REQUIRES driver  # <- questo fa sapere a CMake di includere il componente driver, che contiene adc.h
)
