/********************************************
*					    *
* 	allocate.c			    *
*					    *
*					    *
*********************************************

/*==============  global.h  ===============*/

#define PI   3.141592654
#define TRUE  (0==0)
#define FALSE (0==1)
#define EXPORT extern
#define IMPORT extern

#ifndef DOUBLE_PRECISION
#define REAL float
#define FMT  "%f"
#else
#define REAL double
#define FMT  "%lf"
#endif

/*==============  matrix.c  ==================*/

#include <stdio.h>
#include <string.h>
#include "global.h"

REAL *alloc_vector(int len)
{
    return (REAL *) malloc((unsigned) len*sizeof(REAL));

}

double *alloc_vector_double(int len)
{
    return (double *) malloc((unsigned) len*sizeof(double));
}
 

void free_vector_unsigned_char(unsigned char *v)
{
    /*free((unsigned char *) v);*/
    free(( char *) v);
}
  

void free_vector_char(char *v)
{
    free((char *) v);
}


void free_vector_int(int *v)
{
    free( (int *) v);
}


void free_vector(REAL *v)
{
    free( (REAL *) v);
}


void free_vector_double(double *v)
{
    free((double *) v);
}


int *alloc_vector_int(int len)
{
    return (int *) malloc((unsigned) len*sizeof(int));
}

    
unsigned char *alloc_vector_unsigned_char(int len)
{
    return (unsigned char *) malloc((unsigned) len*sizeof(unsigned char));
}


char *alloc_vector_char(int len)
{
    return (char *) malloc((unsigned) len*sizeof( char));
}



char **alloc_matrix_char(int row, int col)
{
    int i;
    char **m;

    m = (char **) malloc((unsigned) row*sizeof(char *));
    for (i=0; i<row; i++)
        m[i] = (char *) malloc((unsigned) col*sizeof(char));
    return m;
}



REAL **alloc_matrix(int row, int col)
{
    int i;
    REAL **m;

    m = (REAL **) malloc((unsigned) row*sizeof(REAL *));
    for (i=0; i<row; i++)
        m[i] = (REAL *) malloc((unsigned) col*sizeof(REAL));
    return m;
}



double **alloc_matrix_double(int row, int col)
{
    int i;
    double **m;

    m = (double **) malloc((unsigned) row*sizeof(double *));
    for (i=0; i<row; i++)
        m[i] = (double *) malloc((unsigned) col*sizeof(double));
    return m;
}


int **alloc_matrix_int(int row, int col)
{
    int i;
    int **m;

    m = (int **) malloc((unsigned) row*sizeof(int *));
    for (i=0; i<row; i++)
        m[i] = (int *) malloc((unsigned) col*sizeof(int));
    return m;
}



REAL ***alloc_hipper_matrix(int z, int row, int col)
{
    int i;
    REAL ***m;

    m = (REAL ***) malloc((unsigned) z*sizeof(REAL *));
    for (i=0; i<z; i++)
        m[i] = (REAL **) alloc_matrix(row,col);
    return m;
}


REAL ****alloc_four(int t, int z, int row, int col)
{
    int i;
    REAL ****m;
    //REAL ***alloc_hipper_matrix();

    m = (REAL ****) malloc((unsigned) t*sizeof(REAL *));
    for (i=0; i<t; i++)
        m[i] = (REAL ***) alloc_hipper_matrix(z,row,col);
    return m;
}






double ***alloc_hipper_double(int z, int row, int col)
{
    int i;
    double ***m;

    m = (double ***) malloc((unsigned) z*sizeof(double *));
    for (i=0; i<z; i++)
        m[i] = (double **) alloc_matrix_double(row,col);
    return m;
}
    

int ***alloc_hipper_int(int z,int row,int col)
{
    int i;
    int ***m;

    m = (int ***) malloc((unsigned) z*sizeof(int *));
    for (i=0; i<z; i++)
        m[i] = (int **) alloc_matrix_int(row,col);
    return m;
}


 
char ***alloc_hipper_char(int z,int row,int col)
{
    int i;
    char ***m;

    m = (char ***) malloc((unsigned) z*sizeof(char *));
    for (i=0; i<z; i++)
        m[i] = (char **) alloc_matrix_char(row,col);
    return m;
}


void free_matrix(REAL **m,int row)
{
    int i;

    for (i=0; i<row; i++)
        free( (REAL *) m[i]);
    free((REAL *) m);
    /*
    for (i=0; i<row; i++)
        free((char *) m[i]);
    free((char *) m);*/
}



void free_matrix_double(double **m,int row)
{
    int i;

    for (i=0; i<row; i++)
        free((char *) m[i]);
    free((char *) m);
}




void free_matrix_int(int **m,int row)
{
    int i;

    for (i=0; i<row; i++)
        free((char *) m[i]);
    free((char *) m);
}




void free_matrix_char(char **m,int row)
{
    int i;

    for (i=0; i<row; i++)
        free((char *) m[i]);
    free((char *) m);
}


void free_matrix_hipper(REAL ***m,int z,int row)
{
    int i;

    for (i=0; i<z; i++)
        free_matrix( m[i],row);
}

void free_four(REAL ****m,int t,int z,int row)
{
    int i;
    //void free_matrix_hipper();

    for (i=0; i<t; i++)
        free_matrix_hipper(m[i],z,row);
}





void free_hipper_double(double ***m,int z,int row)
{
    int i;

    for (i=0; i<z; i++)
        free_matrix_double( m[i],row);
}


  
void free_hipper_int(int ***m,int z,int row)
{
    int i;

    for (i=0; i<z; i++)
        free_matrix_int( m[i],row);
}


 
void free_hipper_char(char ***m,int z,int row)
{
    int i;

    for (i=0; i<z; i++)
        free_matrix_int((int **)  m[i],row);
}



void copy_vector(int len,REAL *src,REAL *dst)
{
    int i;
    for (i=0; i<len; i++) dst[i] = src[i];
}



void add_vector(int len,REAL *src1,REAL *src2,REAL *dst)
{
    int i;
    for (i=0; i<len; i++) dst[i] = src1[i] + src2[i];
}



void sub_vector(int len,REAL *src1,REAL *src2,REAL *dst)
{
    int i;
    for (i=0; i<len; i++) dst[i] = src1[i] - src2[i];
}



void copy_matrix (int row, int col, REAL **src, REAL **dst)
{
    int r, c;
    for (r=0; r<row; r++)
	for (c=0; c<col; c++)
	    dst[r][c] = src[r][c];
}



void add_matrix (int row, int col, REAL **src1, REAL **src2, REAL **dst)
{
    int r, c;
    for (r=0; r<row; r++)
        for (c=0; c<col; c++)
            dst[r][c] = src1[r][c] + src2[r][c];
}



void sub_matrix (int row, int col, REAL **src1, REAL **src2, REAL **dst)
{
    int r, c;
    for (r=0; r<row; r++)
        for (c=0; c<col; c++)
            dst[r][c] = src1[r][c] - src2[r][c];
}


/* ------------  File I/O Routines  -------------------- */

#include <stdio.h>

#define BLOCKSIZE 1024

REAL *read_vector(char *filename,int *len)
{
    FILE *fp;
    int i,count,blk,done;
    REAL *dat, *ptr;

    if ((fp = fopen(filename,"r")) == NULL) {
	printf("Could not open file: %s\n",filename);
	exit(1);
    }
    done = FALSE;
    blk = count = 0;
    dat = (REAL *) malloc(sizeof(REAL));
    while (!done) {
	ptr = (REAL *) realloc(dat, (++blk) * BLOCKSIZE * sizeof(REAL));
	if (ptr == NULL) {
	    printf ("Could not allocate enough memory!\n");
	    exit(1);
	}
	dat = ptr;
        for (i=0; i<BLOCKSIZE && !done; i++) {
            fscanf(fp,FMT,&dat[count]);
            if (feof(fp)) done = TRUE;
            else          count++;
        }
    }
    fclose(fp);
    ptr = (REAL *) realloc(dat, count*sizeof(REAL));
    if (ptr == NULL) {
        printf ("Could not allocate enough memory!\n");
        exit(1);
    }
    *len = count;
    return dat;
}



int *read_int_vector(char *filename,int len,int *dat)
{
    FILE *fp;
    int i,count,blk,done;
    int  *ptr;
    int *alloc_vector_int();

    if ((fp = fopen(filename,"r")) == NULL) {
        printf("Could not open file: %s\n",filename);
        exit(1);
    }

    fread(dat,sizeof(int),len,fp);
    /*for(i=1;i<= len;i++){
        printf("\n ob[%d] = %d",i,dat[i]);
    }*/

    fclose(fp);
}
     

     
REAL **read_matrix (char *filename, int *row, int *col)
{
    FILE *fp;
    char *buffer;
    REAL *vec, **dat;
    int  i, r, c, done, blk, count;

    if ((fp = fopen(filename,"r")) == NULL) {
	printf("Could not open file: %s\n",filename);
	exit(1);
    }
    fscanf (fp, "%d%d", row, col);
    dat = (REAL **) alloc_matrix (*row, *col);

    for (r=0; r<*row; r++)
	for (c=0; c<*col; c++){
	    fscanf(fp,FMT,&dat[r][c]);
	    printf("dat[%d][%d] %f\n",r,c,dat[r][c]);
	}
    fclose(fp);
    return dat;

}



void write_vector(char *filename, int len, REAL *dat)
{
    FILE *fp;
    int i;

    fp = fopen(filename,"w");
    for (i=0; i<len; i++) {
        fprintf(fp,"%13.6f\n",dat[i]);
    }
    fclose(fp);
}



void write_vector_raw(char *filename, int len, REAL *dat)
{
    FILE *fp;

    fp = fopen(filename,"w");
    fwrite(dat,sizeof(REAL),len,fp);
    fclose(fp);
}



void write_int_vector_raw(char *filename, int len,int *dat)
{
    FILE *fp;

    fp = fopen(filename,"w");
    if(fp==0){
	printf("/n Can not open %s ",filename);
	exit(0);
    }

    fwrite(dat,sizeof(int),len,fp);
    fclose(fp);
}



void write_unsigned_char_vector_raw(char *filename, int len, unsigned char *dat)
{
    FILE *fp;

    fp = fopen(filename,"w");
    if(fp==0){
        printf("/n Can not open %s ",filename);
        exit(0);
    }

    fwrite(dat,sizeof(unsigned char),len,fp);
    fclose(fp);
}



void write_matrix (char *filename, int row, int col, REAL **dat)
{
    FILE *fp;
    int  r,c;

    fp = fopen (filename, "w");
    fprintf (fp,"%d %d\n",row, col);
    for (r=0; r<row; r++) {
	for (c=0; c<col; c++) {
	    fprintf (fp, "%13.6f\n", dat[r][c]);
	}
    }
    fclose (fp);
}



void write_matrix_raw (char *filename, int row, int col, REAL **dat)
{
    FILE *fp;
    int r;

    fp = fopen (filename, "w");
    for (r=0; r<row; r++){
	/*printf("\n row %d");*/
	fwrite (dat[r], sizeof(REAL), col, fp);
    }

    fclose (fp);
}



void read_matrix_raw(char *filename, int row, int col, float **dat)
{
    FILE *fp;
    int r;
    char file[200];

    strcpy(file,filename);
    fp=fopen(file,"r");
    if(fp==0){
             printf("\n Can not open %s ",filename);
             exit(0);
    }
    for (r=0; r<row; r++){
        /*printf("\n row %d\n",r);*/
        fread (dat[r], sizeof(REAL), col, fp);
    }
    fclose (fp);
}
