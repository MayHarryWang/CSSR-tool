# **CSSR-tool**
## **Overview**

CSSR-tool converts all source CSSR messages into one text file. The file accumulates QZSS MADOCA, Galileo HAS and BDS-3 B2b messages with its corresponding GPS time, PRN number and HEX code. It can be utilized as an input of coorperative CSSR-PPP. Users can arbitrarily select which service should be included in the output text file.

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

<img src="image/SSR_Tool.png" width=100%>

--------------------------------------------------------------------------------

## **Contents**
```
CSSR-tool --+-- src     CSSR-tool C programs
            |   +--     rtklib.h (Inherited from RTKlib) 
            |   +--     cssr.h
            |   +--     pksdr_mt.h
            |   +--     sbf_mt.h
            |   +--     gflib.h
            |   +--     cssr_main.c
            |   +--     cssr_bds.c
            |   +--     cssr_has.c
            |   +--     cssr_mdc.c
            |   +--     decode_sbf.c
            |   +--     log_func.c
            |   +--     galois.c
            |   +--     gflib.c
            |   +--     rtkcmn.c (Inherited from RTKlib) 
            |   +--     stream.c (Inherited from RTKlib)
            +-- log     CSSR-tool log file output
            |   +--     CSSR-tool log file link
            +-- data    CSSR-tool sample data link
```
--------------------------------------------------------------------------------

## **Citation**
Under review

## **Contributor**
* **Cheng-Wei Wang** (jlurgbuf1234@gmail.com)

## **References**
[1] https://github.com/tomojitakasu/PocketSDR/tree/master 

[2] https://github.com/tomojitakasu/RTKLIB

[3] https://github.com/hirokawa/cssrlib

[4] https://web.eecs.utk.edu/~jplank/plank/gflib/
