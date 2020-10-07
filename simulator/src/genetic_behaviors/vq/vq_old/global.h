
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
                     
