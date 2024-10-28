//
// Created by User on 2024/1/22.
//
#include "rtklib.h"

#ifdef WIN32
#include <io.h>
#endif
#if defined(AVX2) || defined(AVX512)
#include <immintrin.h>
#endif

#define MIN(x, y)     ((x) < (y) ? (x) : (y))
static int log_lvl = 3;           // log level
static stream_t log_str = {0};    // log stream

// enable escape sequence for Windows console ----------------------------------
static void enable_console_esc(void)
{
#ifdef WIN32
    HANDLE h = (HANDLE)_get_osfhandle(1); // stdout
    DWORD mode = 0;

    if (!GetConsoleMode(h, &mode) ||
        !SetConsoleMode(h, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING)) {
        //fprintf(stderr, "SetConsoleMode() error (%ld)\n", GetLastError());
    }
#endif
}

// open log --------------------------------------------------------------------
int ssr_log_open(const char *path)
{
    const char *p = strchr(path, ':');
    int stat;

    if (!p || *(p + 1) == ':' ) { // file (path = file[::opt...])
        stat = stropen(&log_str, STR_FILE, STR_MODE_W, path);
    }
    else if (p == path) { // TCP server (path = :port)
        stat = stropen(&log_str, STR_TCPSVR, STR_MODE_W, path);
    }
    else { // TCP client (path = addr:port)
        stat = stropen(&log_str, STR_TCPCLI, STR_MODE_W, path);
    }
    if (!stat) {
        fprintf(stderr, "log stream open error %s\n", path);
    }
    return stat;
}

// close log -------------------------------------------------------------------
void ssr_log_close(void)
{
    strclose(&log_str);
}

// set log level ---------------------------------------------------------------
void ssr_log_level(int level)
{
    log_lvl = level;
}

// output log ------------------------------------------------------------------
void ssr_log(int level, const char *msg, ...)
{
    va_list ap;

    va_start(ap, msg);

    if (log_lvl == 0) {
        vprintf(msg, ap);
    }
    else if (level <= log_lvl) {
        char buff[1024];
        int len = vsnprintf(buff, sizeof(buff) - 2, msg, ap);
        len = MIN(len, (int)sizeof(buff) - 3);
        sprintf(buff + len, "\r\n");
        strwrite(&log_str, (uint8_t *)buff, len + 2);
    }
    va_end(ap);
}

// initialize GNSS SDR functions -----------------------------------------------
void ssr_func_init()
{
    // initilize log stream
    strinitcom();
    strinit(&log_str);

    // enable escape sequence for Windows console
    enable_console_esc();
}
