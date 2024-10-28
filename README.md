# **CSSR-tool**
## **Overview**

CSSR-tool converts all source CSSR messages into one text file. The file accumulates QZSS MADOCA, Galileo HAS and BDS-3 B2b messages with its corresponding GPS time, PRN number and HEX code. It can be utilized as an input of coorperative CSSR-PPP. Users can arbitrarily select which service should be included in the output text file.

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

## **References**
[1] https://github.com/tomojitakasu/PocketSDR/tree/master 

[2] https://github.com/tomojitakasu/RTKLIB

[3] https://github.com/hirokawa/cssrlib

[4] https://web.eecs.utk.edu/~jplank/plank/gflib/
