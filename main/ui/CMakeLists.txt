SET(SOURCES screens/ui_Livedata.c
    screens/ui_Screen2.c
    screens/ui_Screen1.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    images/ui_img_jo_png.c
    images/ui_img_115941442.c)

add_library(ui ${SOURCES})
