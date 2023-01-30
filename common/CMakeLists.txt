set(app_src i2c-lcd.c keypad.c Fingerprint.c)
set(pri_req)
idf_component_register(SRCS ${app_src}
                       INCLUDE_DIRS "include"
                       REQUIRES ${pri_req})