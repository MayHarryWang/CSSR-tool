#include <stdio.h>
#include "rtklib.h"
#include "sbf_mt.h"

//------------------------------------------------------------------------------
//
//   Description
//
//     CSSR-tool converts all source CSSR messages into one text file. The file
//     accumulates QZSS MADOCA, Galileo HAS and BDS-3 B2b messages with its corresponding
//     GPS time, PRN number and HEX code. It can be utilized as an input of coorperative
//     CSSR-PPP. Users can arbitrarily choose which service should be included in
//     the text file:
//
//     Default: log_b2b=1, log_has=1, log_mdc=1;
//     If you only want to output B2b messages, please set log_has=0, log_mdc=0.
//
//     CSSR-tool inherits several features from PocketSDR and RTKlib demo5 b34.
//     GFlib is used for Galois Field arithmetic. Software description:
//     https://web.eecs.utk.edu/~jplank/plank/gflib/
//
//     This software converts Galileo HAS and BDS-3 B2b from Septentrio SBF file,
//     and QZSS MADOCA from PocketSDR file. Please refer to the format:
//     https://github.com/tomojitakasu/PocketSDR/tree/master
//     The sample file is also included in the repository.
//
//     Note: Users must input SBF file to run the software; however, the PocketSDR file
//     is optional. Besides, this package also supports message contents display.
//     Please refer to the options below.
//
//   Options ([]: default)
//
//     -sbf SBF file path
//         Specify the SBF file path, for example:
//         "data\\20240601\\sept0601.sbf"
//
//     -pksdr PocketSDR file path (extension: .log)
//         Specify the text file output of PocketSDR, for example:
//         "data\\20240601\\pocket_trk_L1L6_0601.log"
//
//     -log Log file output
//         Specify log file output, for example:
//         "log\\sbf_B2bE6BL6E_0601.log"
//
//     -BDSref {59, 60, 61}
//         Specify the reference BDS B2b satellite you want to output or display.
//         Note: only three satellites are available, please select one of them.
//         Default: "59"
//
//     -QZSref {203-209}
//         Specify the reference QZSS satellite you want to output or display.
//         Note: only PRN 203-209 satellites are available, please select one
//         of them.
//         Default: "206"
//
//     -disp {mdc, has, b2b} (optional)
//         Specify the service you want to display the message content. Users
//         can only select one of the service to keep the window readable. The
//         arguments are: "mdc", "has", "b2b". For example:
//         Default: "b2b"
//
//         This is optional. The software will be in silent mode if users do
//         not use the command.
//
//      If the software successfully run, the text "Preprocessing Done!" will
//      be shown at the end of the process.
//
int main(int argc, char **argv)
{
    FILE *fp_sbf = stdin, *fp_mdc = stdin, *fp_has = stdin, *fp_bds = stdin;
    FILE *fp_log;
    char *sbf_file = "", *mdc_file = "", *has_file = "", *bds_file = "";
    char *log_file = "";
    int log_lvl=4, BDS_ref=-1, QZS_ref=-1;
    int disp_b2b = 0, disp_has=0, disp_mdc=0;

    argc = 0;
    int log_b2b=1, log_has=1, log_mdc=1; // Default: log all source messages

    argv[argc++] = "SSR Tool using CLION";

    argv[argc++] = "-sbf";
    argv[argc++] = "data\\20240601\\sept0601.sbf";

    argv[argc++] = "-pksdr";
    argv[argc++] = "data\\20240601\\pocket_trk_L1L6_0601.log";

    argv[argc++] = "-log";
    argv[argc++] = "log\\sbf_B2bE6BL6E_0601.log";

    argv[argc++] = "-BDSref";
    argv[argc++] = "59";

    argv[argc++] = "-QZSref";
    argv[argc++] = "206";

//    argv[argc++] = "-disp";
//    argv[argc++] = "b2b";

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-sbf") && i + 1 < argc) {
            sbf_file = argv[++i];
        }
        else if (!strcmp(argv[i], "-log") && i + 1 < argc) {
            log_file = argv[++i];
        }
        else if (!strcmp(argv[i], "-pksdr") && i + 1 < argc) {
            mdc_file = argv[++i];
        }
        else if (!strcmp(argv[i], "-BDSref") && i + 1 < argc) {
            BDS_ref = atoi(argv[++i]);
        }
        else if (!strcmp(argv[i], "-QZSref") && i + 1 < argc) {
            QZS_ref = atoi(argv[++i]);
        }
        else if (!strcmp(argv[i], "-disp") && i + 1 < argc) {
            ++i;
            if (!strcmp(argv[i], "mdc")) disp_mdc = 1;
            else if (!strcmp(argv[i], "b2b")) disp_b2b = 1;
            else if (!strcmp(argv[i], "has")) disp_has = 1;
        }
    }

    ssr_func_init();
    if (*log_file) {
        ssr_log_open(log_file);
        ssr_log_level(log_lvl);
    }

    if (*sbf_file) {
        if (!(fp_sbf = fopen(sbf_file, "rb"))) {
            fprintf(stderr, "file open error: %s\n", fp_sbf);
            exit(-1);
        }
    }
    if (*mdc_file) {
        if (QZS_ref == -1) {
            fprintf(stderr, "MADOCA file without reference satellite\n");
            exit(-1);
        }
        if (!(fp_mdc = fopen(mdc_file, "r"))) {
            fprintf(stderr, "file open error: %s\n", fp_mdc);
            exit(-1);
        }
    }
    sbf_processor(fp_sbf, fp_mdc, BDS_ref, QZS_ref,
                  disp_b2b, disp_has, disp_mdc,
                  log_b2b,  log_has,  log_mdc);
    ssr_log_close();

    fclose(fp_sbf);
    fclose(fp_mdc);
    printf("Preprocessing Done!\n");
    return 0;
}
