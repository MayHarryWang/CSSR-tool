//
// Created by User on 2024/2/3.
//

#ifndef SSR_TOOL_CLION_GFLIB_H
#define SSR_TOOL_CLION_GFLIB_H
/*
Procedures and Programs for Galois-Field Arithmetic and Reed-Solomon Coding.
Copyright (C) 2003 James S. Plank

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

---------------------------------------------------------------------------
Please see http://www.cs.utk.edu/~plank/plank/gflib
for instruction on how to use this library.

Jim Plank
plank@cs.utk.edu
http://www.cs.utk.edu/~plank

Associate Professor
Department of Computer Science
University of Tennessee
203 Claxton Complex
1122 Volunteer Blvd.
Knoxville, TN 37996-3450

     865-974-4397
Fax: 865-974-4404

$Revision: 1.2 $
*/

#include <stdio.h>
#include <stdlib.h>
#include "rtklib.h"
#define W_8
#ifdef W_8
typedef unsigned char unit;
#elif W_16
typedef unsigned short unit;
#endif

typedef struct {
    int *condensed_matrix;   /* The n*n dispersal matrix with rows deleted */
    int *row_identities;     /* A nx1 vector of the original row identities of the cond_matrix */
} Condensed_Matrix;

extern void gf_modar_setup();
extern int gf_single_multiply(int a, int b);
extern int gf_single_divide(int a, int b);
extern void gf_fprint_matrix(FILE *f, int *m, int rows, int cols);
extern void gf_fast_add_parity(void *to_add, void *to_modify, int size);
extern void gf_add_parity(void *to_add, void *to_modify, int size);
extern void gf_mult_region(void *region, int size, int factor);
extern int gf_log(int value);
extern int *gf_make_vandermonde(int rows, int cols);
extern int *gf_make_dispersal_matrix(int rows, int cols);
extern Condensed_Matrix *gf_condense_dispersal_matrix(
        int *disp,          /* The rows*cols dispersal matrix */
        int *existing_rows, /* A 0/1 column vector -- 1 if the row still exists */
        int rows,
        int  cols);
EXPORT int *gf_invert_matrix(int *mat, int rows);
EXPORT int *gf_matrix_multiply(int *a, int *b, int rows, int cols);  /* Must be square */
extern void gf_write_matrix(FILE *f, int *a, int rows, int cols);
extern int *gf_read_matrix(FILE *f, int *rows, int *cols);

#endif //SSR_TOOL_CLION_GFLIB_H
