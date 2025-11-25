file(GLOB_RECURSE srcs *.c
                    ui/*.c
                   ui/components/*.c
                   ui/screens/*.c
                   ui/images/*.c
                   )
set(include_dirs
    .
    ui
    )
idf_component_register( SRCS ${srcs}
                      INCLUDE_DIRS ${include_dirs}
                      )
idf_component_get_property(lvgl_lib lvgl__lvgl COMPONENT_LIB)
target_compile_options(${lvgl_lib} PRIVATE -Wno-format)