/********************************************************
*	vq_lib.h					*
*							*
*							*
*		University of Mexico U.N.A.M.		*
*			Jesus Savage			*
*							*
*				11/15/2019		*
*				11/15/2019		*
*********************************************************
*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define VQ_DEBUG 
#define EPS 0.0001
#define NUM_MAX_ITERATIONS 512
#define SHOW_DISTORTION 0


int get_size(char *input_file)
{
 int flag;
 int num_vectors;
 char unit2[128];
 FILE *fp2;
 float dummy1, dummy2;
 

 /* It opens the file that contains the data */
 strcpy(unit2,input_file);
 fp2=fopen(unit2,"r");
 if(fp2==0){
             printf("\n Can not open %s ",unit2);
             exit(1);
 }

 flag=1;
 num_vectors=0;
 /* it reads all the file to find the number of points */
 while(flag){    

    if(fscanf(fp2,"%f%f",&dummy1,&dummy2)==EOF){
                        fclose(fp2);
                        flag=0;
    }
    num_vectors++;

 }

 /*printf("\ntotal num_vectors %d\n",num_vectors);*/
 return(num_vectors);

}


 /* it reads the points */
int get_data(char *input_file,float **vectors)
{
 int flag;
 int i;
 char unit2[128];
 FILE *fp2;
 float dummy1, dummy2;


 /* It opens the file that contains the data */
 strcpy(unit2,input_file);
 fp2=fopen(unit2,"r");
 if(fp2==0){
             printf("\n Can not open %s ",unit2);
             exit(0);
 }

 flag=1;
 i=1;
 /* it reads the file */
 while(flag){

    if(fscanf(fp2,"%f%f",&dummy1,&dummy2)==EOF){
                        fclose(fp2);
                        flag=0;
    }
    else{
    	vectors[i][1]=dummy1;
    	vectors[i][2]=dummy2;
	/*printf("\n vectors[%d][1] %f vectors[%d][2] %f",i,vectors[i][1],i,vectors[i][2]);*/
    	i++;
    }

 }

 return(i);

}





double power_2(int num, int times)
{
 int i,time1;
 double y;

 y=(double)num;
 time1=times;
 for(i=1;i<time1;i++){
        y=(double)(y*num);
 }
 return(y);
}

     

float abs1(float x)
{
 float y;
 if(x < 0)y= -x;
 else y=x;
 return(y);
}



/* it gets line inputs */
int get_inputs(int argc, char **argv,int *num_regions, char *in_file, char *out_file,
	      float *epsilon)
{
 int i;

/* It sets default values */
 *epsilon=EPS;
 *num_regions=8;
 strcpy(in_file,"random.dat");
 strcpy(out_file,"vq_regions.dat");

  /* reads input parameters */
  if (argc>1) {
        for(i=1;i<argc;i++){
          if (!strcmp(argv[i],"-h")){
          printf("\n usage: vqs_regions -f random.dat -o vq_regions.dat -m 16 -e 0.001\n"); 
	  exit(0);
          }
          if (!strcmp(argv[i],"-m")){
                *num_regions=atoi(argv[i+1]);
          }
          if (!strcmp(argv[i],"-e")){
                *epsilon=atof(argv[i+1]);
          }
          if (!strcmp(argv[i],"-f")){
                strcpy(in_file,argv[i+1]);
          }
          if (!strcmp(argv[i],"-o")){
                strcpy(out_file,argv[i+1]);
          }
        }
  }
  else{
       printf("\n Using default values");
  }
 
  printf("\n Input '%s' output '%s' num_regions %d epsilon %f\n",
			in_file,out_file,*num_regions, *epsilon);

}


/* gets the distance of the vectors */
double distance_vector(int q, float *vector, float *vector_pattern, double *distance)
{
 int m;
 double difference;

 *distance=0;

 for(m=1;m<=q;m++){

        *distance = *distance + (vector[m]-vector_pattern[m])*(vector[m]-vector_pattern[m]);
        /*printf("\n vector %f vector_patt %f",vector[m],vector_pattern[m]);*/
 }

 *distance=sqrt(*distance);
 /* printf("\n distance %f",*distance);*/

 return(*distance);
}



/* finds the closest lpc pattern to the input correlations */
void find_closest_pattern(float *vector, float **vector_patterns, int q, int num_patterns, double *distance_smallest,
                                int *index_smallest)
{
 int i,j,k,m,jj;
 double dist,smallest;
 int index;

 /* printf("\n 1");*/
 distance_vector(q,vector,vector_patterns[1],&smallest);
 index=1;

 for(i=2;i<num_patterns+1;i++){
        /*printf("\n %d",i);*/
        distance_vector(q,vector,vector_patterns[i],&dist);

        if(dist < smallest){
                smallest=dist;
                index=i;
        }
 }
 *distance_smallest=smallest;
 *index_smallest=index;

}




