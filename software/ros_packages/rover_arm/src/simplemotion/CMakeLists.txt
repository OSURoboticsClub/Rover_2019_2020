add_library(simplemotion2 rs232.c simplemotion.c busdevice.c sm_consts.c)

target_include_directories (simplemotion2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})