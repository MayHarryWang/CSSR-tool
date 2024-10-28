//
// Created by User on 2024/2/3.
//
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
#include "gflib.h"

#define prim_poly_32 020000007
#define prim_poly_16 0210013
#define prim_poly_8 0435
#define prim_poly_4 023
#define prim_poly_2 07
#define d_min_poly              29          // minimal poly
#define d_symsize               8           // symbol size, in bits.
static int gf_already_setup = 0;
#ifdef W_8
static int Modar_w = 8;
static int Modar_nw = 256;
static int Modar_nwm1 = 255;
static int Modar_poly = prim_poly_8;
#elif W_16
static int Modar_w = 16;
static int Modar_nw = 65536;
static int Modar_nwm1 = 65535;
static int Modar_poly = prim_poly_16;
#endif

static int *B_TO_J;
static int *J_TO_B;
static int Modar_M;
static int Modar_N;
static int Modar_Iam;

void gf_modar_setup()
{
    int j, b, t;
    if (gf_already_setup) return;

    B_TO_J = (int *) malloc(sizeof(int)*Modar_nw);
    if (B_TO_J == NULL) {
        perror("gf_modar_setup, malloc B_TO_J");
        exit(1);
    }
    /* When the word size is 8 bits, make three copies of the table so that
       you don't have to do the extra addition or subtraction in the
       multiplication/division routines */

#ifdef W_8
    J_TO_B = (int *)malloc(sizeof(int)*Modar_nw*3);
#elif W_16
    J_TO_B = (int *)malloc(sizeof(int)*Modar_nw);
#endif
    if (J_TO_B == NULL) {
        perror("gf_modar_setup, malloc J_TO_B");
        exit(1);
    }
    for (j = 0; j < Modar_nw; j++) {
        B_TO_J[j] = Modar_nwm1;
        J_TO_B[j] = 0;
    }

    b = 1;
    for (j = 0; j < Modar_nwm1; j++) {
        if (B_TO_J[b] != Modar_nwm1) {
            fprintf(stderr, "Error: j=%d, b=%d, B->J[b]=%d, J->B[j]=%d (0%o)\n",
                    j, b, B_TO_J[b], J_TO_B[j], (b << 1) ^ Modar_poly);
            exit(1);
        }
        B_TO_J[b] = j;
        J_TO_B[j] = b;
        b = b << 1;
        if (b & Modar_nw) b = (b ^ Modar_poly) & Modar_nwm1;
    }
/*   for (j = 0; j < Modar_nw; j++) { */
/*     printf("%3d b->j %3d        %3d j->b %3d\n",  */
/*             j, B_TO_J[j], j, J_TO_B[j]; */
/*   } */

#ifdef W_8
    for (j = 0; j < Modar_nwm1; j++) {
        J_TO_B[j+Modar_nwm1] = J_TO_B[j];
        J_TO_B[j+2*Modar_nwm1] = J_TO_B[j];
    }
    J_TO_B += Modar_nwm1;
#endif
    gf_already_setup = 1;

}

int gf_single_multiply(int xxx, int yyy)
{
    unsigned int sum_j;
    unit zzz;

    gf_modar_setup();
    if (xxx == 0 || yyy == 0) {
        zzz = 0;
    } else {
        sum_j = (int) (B_TO_J[xxx] + (int) B_TO_J[yyy]);
#ifdef W_16
        if (sum_j >= Modar_nwm1) sum_j -= Modar_nwm1;
#endif
        zzz = J_TO_B[sum_j];
    }
    return zzz;
}

int gf_single_divide(int a, int b)
{
    int sum_j;

    gf_modar_setup();
    if (b == 0) return -1;
    if (a == 0) return 0;
    sum_j = B_TO_J[a] - B_TO_J[b];
#ifdef W_16
    if (sum_j < 0) sum_j += Modar_nwm1;
#endif
    return (int) J_TO_B[sum_j];
}


void gf_mult_region(void *region, int size, int factor)
{
    int sum_j;
    int flog;
    unit *r;
    int sz;
    int r_cache;

    gf_modar_setup();

    if (factor == 1) return;
    if (factor == 0) {
        (void) memset(region, 0, size);
        return;
    }

    flog = B_TO_J[factor];
    sz = size / sizeof(unit);
    r = ((unit *) region) + sz;

    while (r != region) {
        r--;
        r_cache = *r;
        if (r_cache != 0) {
            sum_j = (int) (B_TO_J[r_cache] + flog);
#ifdef W_16
            if (sum_j >= Modar_nwm1) sum_j -= Modar_nwm1;
#endif
            *r = J_TO_B[sum_j];
        }
    }
}




void gf_fast_add_parity(void *to_add, void *to_modify, int size)
{
    unsigned long *p, *t;
    int j;

    j = size/sizeof(unsigned long);
    t = (unsigned long *) to_add;
    p = (unsigned long *) to_modify;
    p += j;
    t += j;
    while(t != (unsigned long *) to_add) {
        p--;
        t--;
        *p = *p ^ *t;
    }
}

void gf_add_parity(void *to_add, void *to_modify, int size)
{
    unsigned long ta, tm;
    unsigned char *cta, *ctm;
    int sml;

    if (size <= 0) return;

    ta = (unsigned long) to_add % sizeof(unsigned long);
    tm = (unsigned long) to_modify% sizeof(unsigned long);
    if (ta != tm) {
        fprintf(stderr, "Error: gf_add_parity: to_add and to_modify are not aligned\n");
        exit(1);
    }

    cta = (unsigned char *) to_add;
    ctm = (unsigned char *) to_modify;

    /* Align to long boundary */
    if (ta != 0) {
        while (ta != sizeof(unsigned long) && size > 0) {
            *ctm = *ctm ^ *cta;
            ctm++;
            cta++;
            size--;
            ta++;
        }
    }

    if (size == 0) return;

    /* Call gf_fast_add_parity to do it fast */

    sml = size / sizeof(unsigned long);
    if (sml > 0)  gf_fast_add_parity(cta, ctm, size);
    size -= sml * sizeof(unsigned long);

    if (size == 0) return;

    /* Do the last few bytes if they are unalighed */
    cta += sml * sizeof(unsigned long);
    ctm += sml * sizeof(unsigned long);
    while (size > 0) {
        *ctm = *ctm ^ *cta;
        ctm++;
        cta++;
        size--;
    }
}

extern int gf_log(int value)
{
    return B_TO_J[value];
}

/* This returns the rows*cols vandermonde matrix.  N+M must be
   < 2^w -1.  Row 0 is in elements 0 to cols-1.  Row one is
   in elements cols to 2cols-1.  Etc.*/

//int *gf_make_vandermonde(int rows, int cols)
//{
//    int *vdm, i, j, k;
//    void gf_modar_setup();
//
//    if (rows >= Modar_nwm1 || cols >= Modar_nwm1) {
//        fprintf(stderr, "Error: gf_make_vandermonde: %d + %d >= %d\n",
//                rows, cols, Modar_nwm1);
//        exit(1);
//    }
//
//    vdm = (int *) malloc(sizeof(int) * rows * cols);
//    if (vdm == NULL) {
//        perror("Malloc: Vandermonde matrix");
//        exit(1);
//    }
//    for (i = 0; i < rows; i++) {
//        k = 1;
//        for (j = 0; j < cols; j++) {
//            vdm[i*cols+j] = k;
//            k = gf_single_multiply(k, i);
//        }
//    }
//    return vdm;
//}

static int find_swap_row(int *matrix, int rows, int cols, int row_num)
{
    int j;

    for (j = row_num; j < rows; j++) {
        if (matrix[j*cols+row_num] != 0) return j;
    }
    return -1;
}

//void gf_fprint_matrix(FILE *f, int *matrix, int rows, int cols)
//{
//    int i, j;
//    for (i = 0; i < rows; i++) {
//        for (j = 0; j < cols; j++) {
//            fprintf(f, "%4d", matrix[i*cols+j]);
//        }
//        fprintf(f, "\n");
//    }
//}

//int *gf_make_dispersal_matrix(int rows, int cols)
//{
//    int *vdm, i, j, k, l, inv, tmp, colindex;
//
//    vdm = gf_make_vandermonde(rows, cols);
//
//    for (i = 0; i < cols && i < rows; i++) {
//        j = find_swap_row(vdm, rows, cols, i);
//        if (j == -1) {
//            fprintf(stderr, "Error: make_dispersal_matrix.  Can't find swap row %d\n",
//                    i);
//            exit(1);
//        }
//
///*    printf("\nSwap row: %d\n\n", j);
//    print_matrix(vdm, rows, cols); */
//
//        if (j != i) {
//            for (k = 0; k < cols; k++) {
//                tmp = vdm[j*cols+k];
//                vdm[j*cols+k] = vdm[i*cols+k];
//                vdm[i*cols+k] = tmp;
//            }
//        }
//        if (vdm[i*cols+i] == 0) {
//            fprintf(stderr, "Internal error -- this shouldn't happen\n");
//            exit(1);
//        }
//
///*    printf("\nAfter Swap: %d\n\n", j);
//    print_matrix(vdm, rows, cols); */
//
//        if (vdm[i*cols+i] != 1) {
//            inv = gf_single_divide(1, vdm[i*cols+i]);
//            k = i;
//            for (j = 0; j < rows; j++) {
//                vdm[k] = gf_single_multiply(inv, vdm[k]);
//                k += cols;
//            }
///*      printf("\nAfter multiplying column by : %d\n\n", inv);
//      print_matrix(vdm, rows, cols); */
//
//        }
//        if (vdm[i*cols+i] != 1) {
//            fprintf(stderr, "Internal error -- this shouldn't happen #2)\n");
//            exit(1);
//        }
//
//        for (j = 0; j < cols; j++) {
//            colindex = vdm[i*cols+j];
//            if (j != i && colindex != 0) {
//                k = j;
//                for (l = 0; l < rows; l++) {
//                    vdm[k] = vdm[k] ^ gf_single_multiply(colindex, vdm[l*cols+i]);
//                    k += cols;
//                }
//            }
//        }
//
///*    printf("\nAfter Messing with other columns\n\n", inv);
//    print_matrix(vdm, rows, cols); */
//
//    }
//
//    return vdm;
//}

//extern Condensed_Matrix *gf_condense_dispersal_matrix(
//        int *disp, int *existing_rows, int rows,    int  cols)
//{
//    Condensed_Matrix *cm;
//    int *m;
//    int *id;
//    int i, j, k, tmp;
//
//    /* Allocate cm and initialize */
//    cm = (Condensed_Matrix *) malloc(sizeof(Condensed_Matrix));
//    if (cm == NULL) { perror("gf_condense_dispersal_matrix - Condensed_Matrix"); exit(1); }
//    cm->condensed_matrix = (int *) malloc(sizeof(int)*cols*cols);
//    if (cm->condensed_matrix == NULL) {
//        perror("gf_condense_dispersal_matrix - cm->condensed_matrix");
//        exit(1);
//    }
//    cm->row_identities = (int *) malloc(sizeof(int)*cols);
//    if (cm->row_identities == NULL) {
//        perror("gf_condense_dispersal_matrix - cm->row_identities");
//        exit(1);
//    }
//    m = cm->condensed_matrix;
//    id = cm->row_identities;
//    for (i = 0; i < cols; i++) id[i] = -1;
//
//    /* First put identity rows in their proper places */
//
//    for (i = 0; i < cols; i++) {
//        if (existing_rows[i] != 0) {
//            id[i] = i;
//            tmp = cols*i;
//            for (j = 0; j < cols; j++) m[tmp+j] = disp[tmp+j];
//        }
//    }
//
//    /* Next, put coding rows in */
//    k = 0;
//    for (i = cols; i < rows; i++) {
//        if (existing_rows[i] != 0) {
//            while(k < cols && id[k] != -1) k++;
//            if (k == cols) return cm;
//            id[k] = i;
//            for (j = 0; j < cols; j++) m[cols*k+j] = disp[cols*i+j];
//        }
//    }
//
//    /* If we're here, there are no more coding rows -- check to see that the
//       condensed dispersal matrix is full -- otherwise, it's not -- return an
//       error */
//
//    while(k < cols && id[k] != -1) k++;
//    if (k == cols) return cm;
//
//    free(id);
//    free(m);
//    free(cm);
//    return NULL;
//}

//static pic(int *inv, int *copy, int rows, char *s)
//{
//    printf("\n%s\n\n", s);
//    gf_fprint_matrix(stdout, inv, rows, rows);
//    printf("\n");
//    gf_fprint_matrix(stdout, copy, rows, rows);
//}

extern int *gf_invert_matrix(int *mat, int rows)
{
    int cols, i, j, k, x, rs2;
    int row_start, tmp, inverse;
    int *inv;
    int *copy;
    cols = rows;

    inv = (int *)malloc(sizeof(int)*(cols*rows));
    if (inv == NULL) { perror("gf_invert_matrix - inv"); exit(1); }
    copy = (int *) malloc(sizeof(int)*rows*cols);
    if (copy == NULL) { perror("gf_invert_matrix - copy"); exit(1); }

    k = 0;
    for (i = 0; i < rows; i++) {
        for (j = 0; j < cols; j++) {
            inv[k] = (i == j) ? 1 : 0;
            copy[k] = mat[k];
            k++;
        }
    }

    /* pic(inv, copy, rows, "Start"); */

    /* First -- convert into upper triangular */
    for (i = 0; i < cols; i++) {
        row_start = cols*i;

        /* Swap rows if we ave a zero i,i element.  If we can't swap, then the
           matrix was not invertible */

        if (copy[row_start+i] == 0) {
            for (j = i+1; j < rows && copy[cols*j+i] == 0; j++) ;
            if (j == rows) {
                fprintf(stderr, "gf_invert_matrix: Matrix not invertible!!\n");
                exit(1);
            }
            rs2 = j*cols;
            for (k = 0; k < cols; k++) {
                tmp = copy[row_start+k];
                copy[row_start+k] = copy[rs2+k];
                copy[rs2+k] = tmp;
                tmp = inv[row_start+k];
                inv[row_start+k] = inv[rs2+k];
                inv[rs2+k] = tmp;
            }
        }

        /* Multiply the row by 1/element i,i */
        tmp = copy[row_start+i];
        if (tmp != 1) {
            inverse = gf_single_divide(1, tmp);
            for (j = 0; j < cols; j++) {
                copy[row_start+j] = gf_single_multiply(copy[row_start+j], inverse);
                inv[row_start+j] = gf_single_multiply(inv[row_start+j], inverse);
            }
            /* pic(inv, copy, rows, "Divided through"); */
        }

        /* Now for each j>i, add A_ji*Ai to Aj */
        k = row_start+i;
        for (j = i+1; j != cols; j++) {
            k += cols;
            if (copy[k] != 0) {
                if (copy[k] == 1) {
                    rs2 = cols*j;
                    for (x = 0; x < cols; x++) {
                        copy[rs2+x] ^= copy[row_start+x];
                        inv[rs2+x] ^= inv[row_start+x];
                    }
                } else {
                    tmp = copy[k];
                    rs2 = cols*j;
                    for (x = 0; x < cols; x++) {
                        copy[rs2+x] ^= gf_single_multiply(tmp, copy[row_start+x]);
                        inv[rs2+x] ^= gf_single_multiply(tmp, inv[row_start+x]);
                    }
                }
            }
        }
        /* pic(inv, copy, rows, "Eliminated rows"); */
    }

    /* Now the matrix is upper triangular.  Start at the top and multiply down */

    for (i = rows-1; i >= 0; i--) {
        row_start = i*cols;
        for (j = 0; j < i; j++) {
            rs2 = j*cols;
            if (copy[rs2+i] != 0) {
                tmp = copy[rs2+i];
                copy[rs2+i] = 0;
                for (k = 0; k < cols; k++) {
                    inv[rs2+k] ^= gf_single_multiply(tmp, inv[row_start+k]);
                }
            }
        }
        /* pic(inv, copy, rows, "One Column"); */
    }
    free(copy);
    return inv;
}

extern int *gf_matrix_multiply(int *a, int *b, int rows, int cols)
{
    int *prod, i, j, k;

    prod = (int *) malloc(sizeof(int)*rows*cols);
    if (prod == NULL) { perror("gf_matrix_multiply - prod"); exit(1); }

    for (i = 0; i < rows*cols; i++) prod[i] = 0;

    for (i = 0; i < rows; i++) {
        for (j = 0; j < cols; j++) {
            for (k = 0; k < cols; k++) {
                prod[i*cols+j] ^= gf_single_multiply(a[i*cols+k],b[k+cols*j]);
            }
        }
    }
    return prod;
}

//void gf_write_matrix(FILE *f, int *a, int rows, int cols)
//{
//    int i;
//    fprintf(f, "%d\n%d\n", rows, cols);
//    for (i = 0; i < rows*cols; i++) fprintf(f, "%d\n", a[i]);
//}
//
//int *gf_read_matrix(FILE *f, int *rows, int *cols)
//{
//    int i, r, c, *a;
//
//    if (fscanf(f, "%d", &r) == 0) {
//        fprintf(stderr, "ERROR reading file -- rows\n");
//        exit(1);
//    }
//    if (fscanf(f, "%d", &c) == 0) {
//        fprintf(stderr, "ERROR reading file -- cols\n");
//        exit(1);
//    }
//    a = (int *) malloc(sizeof(int)*r*c);
//    if (a == NULL) { perror("gf_read_matrix: malloc"); exit(1); }
//
//    for (i = 0; i < r*c; i++) {
//        if (fscanf(f, "%d", a+i) == 0) {
//            fprintf(stderr, "ERROR reading file -- element %d\n", i);
//            exit(1);
//        }
//    }
//    *rows = r;
//    *cols = c;
//    return a;
//}
