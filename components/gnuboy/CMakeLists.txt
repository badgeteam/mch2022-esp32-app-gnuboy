idf_component_register(
  SRCS
    "cpu.c"
    "events.c"
    "hw.c"
    "lcd.c"
    "main.c"
    "palette.c"
    "rcfile.c"
    "refresh.c"
    "sound.c"
    "debug.c"
    "exports.c"
    "inflate.c"
    "lcdc.c"
    "mem.c"
    "path.c"
    "rckeys.c"
    "rtc.c"
    "split.c"
    "emu.c"
    "fastmem.c"
    "keytable.c"
    "loader.c"
    "newsound.c"
    "rccmds.c"
    "rcvars.c"
    "save.c"
  INCLUDE_DIRS .
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE "-DGNUBOY_NO_MINIZIP -DGNUBOY_NO_SCREENSHOT -DIS_LITTLE_ENDIAN -w")

