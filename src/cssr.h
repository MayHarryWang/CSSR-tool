//
// Created by User on 2024/1/26.
//

#ifndef SSR_TOOL_CLION_CSSR_H
#define SSR_TOOL_CLION_CSSR_H

#include <stdint.h>
#include <stdio.h>
#include "rtklib.h"

#ifdef WIN_DLL
#define EXPORT __declspec(dllexport) /* for Windows DLL */
#else
#define EXPORT
#endif

#define CSSR_SYS_GPS    0
#define CSSR_SYS_GLO    1
#define CSSR_SYS_GAL    2
#define CSSR_SYS_BDS    3
#define CSSR_SYS_QZS    4
#define CSSR_SYS_SBS    5

#endif //SSR_TOOL_CLION_CSSR_H
