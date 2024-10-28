cmake_minimum_required(VERSION 3.27)
project(CSSR-tool C)

set(CMAKE_C_STANDARD 11)
set(LINK_DIR D:/CSSR-tool)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

link_libraries(winmm)
link_libraries(ws2_32)

add_executable(CSSR-tool cssr_main.c
        src/decode_sbf.c
        src/log_func.c
        src/rtkcmn.c
        src/stream.c
        src/cssr_bds.c
        src/cssr.h
        src/cssr_has.c
        src/galois.c
        src/gflib.c
        src/gflib.h
        src/cssr_mdc.c
)

#file(GLOB files *.c)
#foreach (file ${files})
#    string(REGEX REPLACE ".+/(.+)\\..*" "\\1" exe ${file})
#    add_executable (${exe} ${file})
#    message (\ \ \ \ --\ src/${exe}.cpp\ will\ be\ compiled\ to\ bin/${exe})
#endforeach ()