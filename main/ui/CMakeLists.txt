SET(SOURCES screens/ui_Livedata.c
    screens/ui_Screen2.c
    screens/ui_Screen1.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    images/ui_img_jo_png.c
    images/ui_img_111926670.c
    images/ui_img_plus_png.c
    images/ui_img_minus_png.c
    images/ui_img_imiev_png.c
    images/ui_img_imiev_charge_png.c)

add_library(ui ${SOURCES})
