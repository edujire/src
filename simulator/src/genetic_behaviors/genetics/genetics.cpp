
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
//#include "/usr/include/sys/time.h"

#define NUM_BEST 2
#define MAX_NUMBER_IND 200
#define PERCENTAGE_CROSS .30  // percentage of bits crossed if there is a cross between two individuals 
//#define PERCENTAGE_MUT .10  // percentage of bits mutated if there is a mutation
#define PERCENTAGE_MUT .20  // percentage of bits mutated if there is a mutation
//#define PERCENTAGE_MUT .1 // percentage of bits mutated if there is a mutation
#define CROSS_TYPE 3 // Crossing type 1: region z1 and z2, 2: Swap z1; 3: 2 regions z1 and z2, z2 and z4 
#define NUM_MAX_STATES 32
#define NUM_MAX_INPUTS 256
#define NUM_MAX_OUTPUTS 10
#define MAX_TYPE 20
#define ROW_PRB 3
#define CLM_PRB 3
#define ROW_RWD 6
#define CLM_RWD 6
#define NUM_MDP_PRB 8


typedef struct _cromosome_binary{
        float fitness;
        int *bits;
} cromosome_binary;

typedef struct _cromosome{
        float fitness;
    int **state;
  int **output;
        float **variables;
} cromosome;
typedef struct _cromosome_potential{
        float fitness;
        float *variables;
} cromosome_potential;


int *alloc_int(int row)
{
    int *m;

    m = (int *) malloc((unsigned) row*sizeof(int));
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
float *alloc_float(int row)
{
    float *m;

    m = (float *) malloc((unsigned) row*sizeof(float));
    return m;
}


float **alloc_matrix_float(int row, int col)
{
    int i;
    float **m;

    m = (float **) malloc((unsigned) row*sizeof(float*));
    for (i=0; i<row; i++)
        m[i] = (float *) malloc((unsigned) col*sizeof(float));
    return m;
}
 //IT GENERATES RANDOM NUMBERS
float generaR(float LMin,float LMax){
  float n;
  int BIG= 0x7FFFFFFF;

  n=LMin+ ((float) random()/(float) BIG ) *(LMax-LMin);
  return n;
}

void sort(int num_individuals,cromosome_binary *old_pop,int num_bits_individuals){

 int i,j,k,l;
 int tmp;
 float ftmp;


 for(l=0;l<num_individuals;l++){

        ftmp=old_pop[l].fitness;

        for(k=l+1;k<num_individuals;k++){
                if (ftmp < old_pop[k].fitness){

                        ftmp=old_pop[k].fitness;
                        old_pop[k].fitness=old_pop[l].fitness;
                        old_pop[l].fitness=ftmp;


                        for(i=0;i<num_bits_individuals;i++){
                               tmp=old_pop[k].bits[i];
                               old_pop[k].bits[i]= old_pop[l].bits[i];
                               old_pop[l].bits[i] = tmp;
                        }

                }

        }

    printf("%d fitness %f\n",l,old_pop[l].fitness);

 }

}

 // it reads the file's population and its performance 
void  read_pop_binary(char *path, char *behavior, cromosome_binary *old_pop, int num_individuals, int num_bits_individuals){

 FILE *fpr,*fpp;
 char file_population[300];
 char file_fitness[300];
 int i,j,k;
 int num_states,num_output,num_inputs;
 int x;
 float fitness;

 // It opens the population file	
 sprintf(file_population,"%savoid_%s.dat",path,behavior);
 if ((fpp=fopen(file_population,"r"))==NULL){
 	     	printf("\nError opening file %s\n",file_population);
      		exit(1);
 }
 printf("\n\nRead population %s\n",file_population);

// It opens the population fitness
 sprintf(file_fitness,"%sfitness_%s.dat",path,behavior);
 if ((fpr=fopen(file_fitness,"r"))==NULL){
                printf("Error opening file %s\n",file_fitness);
                exit(1);
 }


 for(k=0;k<num_individuals;k++){

	// It reads the fitness for each individual
	fscanf(fpr,"%f",&fitness);
	old_pop[k].fitness=fitness;
        //printf("\n%s fitness %f\n",file_fitness,old_pop[k].fitness);

	//printf("Old population\n");
       	for(i=0;i<num_bits_individuals;i++){

                	if(fscanf(fpp,"%d",&x) == EOF)
                       	 	break;
                	else{
                        	old_pop[k].bits[i]= x;
        			//printf("%d ",old_pop[k].bits[i]);
                	}
       	}
	//printf("\n");

        //printf("%s fitness %f\n",file_fitness,old_pop[k].fitness);
 }

 fclose(fpp);
 fclose(fpr);

}
void  read_pop_binary_transform(char *file_name, cromosome_binary *old_pop, int num_individuals, int num_bits_individuals){

 FILE *fpp;
 char file_population[300];
 char file_fitness[300];
 int i,j,k;
 int num_states,num_output,num_inputs;
 int x;
 float distance;
 int num_stuck;
 int num_tries;
 int num_backward;
 float fitness;

 // It opens the population file        
 sprintf(file_population,"%s.dat",file_name);
 if ((fpp=fopen(file_population,"r"))==NULL){
                printf("\nError opening file %s\n",file_population);
                exit(1);
 }
 printf("\n\nRead population %s\n",file_population);
 printf("num_individuals %d num_bits_individuals %d\n",num_individuals,num_bits_individuals);



 for(k=0;k<num_individuals;k++){
  //printf("individual %d\n",k);
        for(i=0;i<num_bits_individuals;i++){

                        if(fscanf(fpp,"%d",&x) == EOF)
                                break;
                        else{
                                old_pop[k].bits[i]= x;
                                //printf("%d ",old_pop[k].bits[i]);
                        }
        }
        //printf("\n");

 }

 fclose(fpp);

}
void transform(cromosome_binary *old_pop, cromosome *new_pop, int num_bits_states, int num_bits_inputs, int num_bits_output, int num_individuals){

  int i,j;
  int k,l;

  for(k=0;k<num_individuals;k++){
    i=0;
    printf("Individuals %d \n",k);
    for(j=0;j < (2<<(num_bits_states+num_bits_inputs-1)); j++){
        for (l=0;l<num_bits_states;l++){
                new_pop[k].state[j][l]=old_pop[k].bits[i];
    //printf("%d ",new_pop[k].state[j][l]);
    i++;
        }
        for (l=0;l<num_bits_output;l++){
                new_pop[k].output[j][l]=old_pop[k].bits[i];
                //printf("%d ",new_pop[k].output[j][l]);
    i++;
        }
  //printf("\n");
    }
  }

}
void fprint_state_output(int index, int num_bits_states, int num_bits_inputs, int num_bits_output, cromosome *pop, char *file_individual){

  int i,j;
  FILE *fp;

  if ((fp=fopen(file_individual,"w"))==NULL){
                fprintf(stderr,"Error opening file %s\n",file_individual);
                exit(1);
  }

  printf("\n\npop %s\n",file_individual);
  fprintf(fp,"%d %d %d\n",num_bits_states,num_bits_output,num_bits_inputs);
  //printf("%d %d %d\n",num_bits_states,num_bits_output,num_bits_inputs);

  for(j=0;j < (2<<(num_bits_states+num_bits_inputs-1)); j++){
        for (i=0;i<num_bits_states;i++){
                fprintf(fp,"%d ",pop[index].state[j][i]);
                //printf("%d ",pop[index].state[j][i]);
        }
        for (i=0;i<num_bits_output;i++){
                fprintf(fp,"%d ",pop[index].output[j][i]);
                //printf("%d ",pop[index].output[j][i]);
        }
        fprintf(fp,"\n");
        //printf("\n");
  }

  fclose(fp);

}

void transform_state_machine(char *path, int num_individuals, int num_bits_individuals){

  int num_bits_states,num_bits_output,num_bits_inputs;
  FILE *fp;
  char file_name[300],file_individual[300],file_name_old[300];
  int i,j,k;
  cromosome new_pop[MAX_NUMBER_IND];
  cromosome_binary old_pop[MAX_NUMBER_IND];
  int num_best=NUM_BEST;
  float num_best_factor;


  // It gets the argument data
  if (num_individuals> MAX_NUMBER_IND) {
    printf("Increase MAX_NUMBER_IND in file genetics.h \n");
    exit(1);
  }

  num_bits_states=4;
  num_bits_output=3;
  num_bits_inputs=8;
  strcpy(file_name,path);
  strcpy(file_name_old,path);
  strcat(file_name_old,"old_");
  strcat(file_name,"avoid_fsm");
  strcat(file_name_old,"avoid_fsm");



  for(i=0;i< num_individuals;i++){
        old_pop[i].bits =  (int *) alloc_int(num_bits_individuals+1);
  }

  for(i=0;i< num_individuals;i++){
        new_pop[i].state =  (int **) alloc_matrix_int((2<<(num_bits_states+num_bits_inputs+1)),num_bits_states);
        new_pop[i].output =  (int **) alloc_matrix_int((2<<(num_bits_states+num_bits_inputs+1)),num_bits_output);
  }


  //printf("Read the file with population %s \n",file_name);
  read_pop_binary_transform(file_name,old_pop,num_individuals,num_bits_individuals);

  // it transforms the bits to an structure that the state machine uses  
  transform(old_pop,new_pop,num_bits_states,num_bits_inputs,num_bits_output,num_individuals);

  for(i=0;i<num_individuals;i++){
        // it saves the new population
        sprintf(file_individual,"%s_%d.dat",file_name,i);
        fprint_state_output(i,num_bits_states,num_bits_inputs,num_bits_output,new_pop,file_individual);

  }

}
void fprint_state_output_hmm(int index,cromosome_potential *pop, char *file_individual,int num_bits_inputs){

  int i,j;
  int num_states,num_symbols,num_outputs;
  float sum_prb;
  FILE *fp;
  int m,n;


  if ((fp=fopen(file_individual,"w"))==NULL){
                fprintf(stderr,"Error opening file %s\n",file_individual);
                exit(1);
  }

  num_states= pop[index].variables[0];
  num_symbols= pop[index].variables[1];
  num_outputs= pop[index].variables[2];



  //printf("\n\npop %s\n",file_individual);

  fprintf(fp,"%d\n",num_states);
  //printf("Num_states %d\n",num_states);
  fprintf(fp,"%d\n",num_symbols);
  //printf("Num_symbols %d\n",num_symbols);
  fprintf(fp,"%d\n\n",num_outputs);
  //printf("Num_outputs %d\n",num_outputs);

   m=3;
   // it saves pi
   sum_prb=0.0;
   for(n=1; n <= num_states; n++){
  fprintf(fp,"%f ",pop[index].variables[m]);
  //printf(" pop. %d pi[%d] %f\n",index,n,pop[index].variables[m]);
  sum_prb=sum_prb+pop[index].variables[m];
  m++;
   }
   fprintf(fp,"\n\n");
   //printf(" sum. prb Pi %f\n\n",sum_prb);

   // it saves the A matrix
  for(j=1; j <= num_states; j++){
    sum_prb=0.0;
  for(n=1; n <= num_states; n++){
    fprintf(fp,"%f ",pop[index].variables[m]);
          //printf(" pop. %d a[%d][%d] %f\n",index,j,n,pop[index].variables[m]);
          sum_prb=sum_prb+pop[index].variables[m];
    m++;
    }
    fprintf(fp,"\n");
    //printf(" sum. prb A %d %f\n\n",j,sum_prb);

  }
  fprintf(fp,"\n");


  // it saves the B matrix
  for(j=1; j <= num_states; j++){
        sum_prb=0.0;
        for(n=1; n <= num_symbols; n++){
                fprintf(fp,"%f ",pop[index].variables[m]);
                //printf(" pop. %d b[%d][%d] %f\n",index,j,n,pop[index].variables[m]);
                sum_prb=sum_prb+pop[index].variables[m];
    m++;
        }
        fprintf(fp,"\n");
    //printf(" sum. prb B %d %f\n\n",j,sum_prb);
  }
  fprintf(fp,"\n");

    // it saves the C matrix
  for(j=1; j <= num_states*num_symbols; j++){
        sum_prb=0.0;
        for(n=1; n <= num_outputs; n++){
                fprintf(fp,"%f ",pop[index].variables[m]);
                //printf(" pop. %d C[%d][%d] %f\n",index,j,n,pop[index].variables[m]);
                sum_prb=sum_prb+pop[index].variables[m];
    m++;
        }
        fprintf(fp,"\n");
        //printf(" sum. prb C %d %f\n\n",j,sum_prb);
  }


  fclose(fp);

}
void  transform_pts_hmm(cromosome_binary *old_pop,cromosome_potential *new_pop, int num_variables,
    int num_bits_variables, int num_individuals, int num_states, int num_outputs, int num_symbols){

  int i,j;
  int k,l;
  int bits[100];
  float var,frac2=1.0;
  int m=0;
  //int num_states,num_inputs,num_outputs;
  int vari;
  float sum;
  int ini,n,nn;
  float sum_prb;




  for(k=0;k<num_individuals;k++){
    printf("Individual %d \n",k);

  // it gets first num_states
    i=0;

    new_pop[k].variables[0]=num_states;
  //printf("\nnum_states %d\n",num_states);
  if(num_states > NUM_MAX_STATES){
    printf("Increse NUM_MAX_STATES > %d in genetics.h \n",num_states);
    exit(0);
  }

  printf("num_bits_variables %d\n",num_bits_variables);
    
    new_pop[k].variables[1]=num_symbols;
  //printf("Num. Symbols %d\n",num_symbols);

    new_pop[k].variables[2]=num_outputs;

  //printf("\nnum_outputs %d\n",num_outputs);
  if(num_outputs > NUM_MAX_OUTPUTS){
    printf("Increse NUM_MAX_OUTPUTS > %d in genetics.h \n",num_outputs);
    exit(0);
  }

  printf("\n\nNum_states %d num_symbols %d num_outputs %d\n",num_states,num_symbols,num_outputs);

 
  // it gets the components of Pi
  m=3;
  sum=0.0;
  ini=m-1;

  for(n=1; n <= num_states; n++){
          for(j=0;j < num_bits_variables; j++){
                  bits[j]=old_pop[k].bits[i];
                  //printf("%d",bits[j]);
                  i++;
          }
 
          var=0;
          for(j=num_bits_variables-1,l=0;j >= 1; j--,l++){
                  var=bits[j]*(2 << l)+var;
          }

          var= var/frac2 + 0.001 ;
    sum=sum+var;
          new_pop[k].variables[m]=var;
          //printf(" pop. %d var[%d] %f\n ",k,m,new_pop[k].variables[m]);
    m++;
  }


  sum_prb=0.0;
  // normalization
  for(n=1; n <= num_states; n++){
          new_pop[k].variables[ini+n]=new_pop[k].variables[ini+n]/sum;
    sum_prb= sum_prb + new_pop[k].variables[ini+n];
          printf(" pop. %d pi[%d] %f\n",k,n,new_pop[k].variables[ini+n]);
    
  }
  printf(" sum. prb Pi %f\n\n",sum_prb);

  // it gets the components of A
        for(nn=1; nn <= num_states; nn++){
         
   ini=m-1;
         sum=0.0;
         
   for(n=1; n <= num_states; n++){


                for(j=0;j < num_bits_variables; j++){
                        bits[j]=old_pop[k].bits[i];
                        //printf("%d",bits[j]);
                        i++;
                }

                var=0;
                for(j=num_bits_variables-1,l=0;j >= 1; j--,l++){
                        var=bits[j]*(2 << l)+var;
                }

                var= var/frac2 + 0.001 ;
                sum=sum+var;
                new_pop[k].variables[m]=var;
                //printf(" pop. %d var[%d] %f\n ",k,m,new_pop[k].variables[m]);
                m++;
         }


         sum_prb=0.0;
         // normalization
         for(n=1; n <= num_states; n++){
                new_pop[k].variables[ini+n]=new_pop[k].variables[ini+n]/sum;
                sum_prb= sum_prb + new_pop[k].variables[ini+n];
                printf(" pop. %d a[%d][%d] %f\n",k,nn,n,new_pop[k].variables[ini+n]);

         }
         printf(" sum. prb A %d %f\n\n",nn,sum_prb);

  }


  // it gets the components of B
        for(nn=1; nn <= num_states; nn++){
         
         ini=m-1;
         sum=0.0;
         
         for(n=1; n <= num_symbols; n++){


                for(j=0;j < num_bits_variables; j++){
                        bits[j]=old_pop[k].bits[i];
                        //printf("%d",bits[j]);
                        i++;
                }

                var=0;
                for(j=num_bits_variables-1,l=0;j >= 1; j--,l++){
                        var=bits[j]*(2 << l)+var;
                }

                var= var/frac2  + 0.001;
                sum=sum+var;
                new_pop[k].variables[m]=var;
                //printf(" pop. %d var[%d] %f\n ",k,m,new_pop[k].variables[m]);
                m++;
         }


         sum_prb=0.0;
         // normalization
         for(n=1; n <= num_symbols; n++){
                new_pop[k].variables[ini+n]=new_pop[k].variables[ini+n]/sum;
                sum_prb= sum_prb + new_pop[k].variables[ini+n];
                printf(" pop. %d b[%d][%d] %f\n",k,nn,n,new_pop[k].variables[ini+n]);

         }
         printf(" sum. prb B %d %f\n\n",nn,sum_prb);

        }

   // it gets the components of C
        for(nn=1; nn <= num_states*num_symbols; nn++){

         ini=m-1;
         sum=0.0;

         for(n=1; n <= num_outputs; n++){

                for(j=0;j < num_bits_variables; j++){
                        bits[j]=old_pop[k].bits[i];
                        //printf("%d",bits[j]);
                        i++;
                }

                var=0;
                for(j=num_bits_variables-1,l=0;j >= 1; j--,l++){
                        var=bits[j]*(2 << l)+var;
                }

                var= var/frac2  + 0.001;
                sum=sum+var;
                new_pop[k].variables[m]=var;
                //printf(" pop. %d var[%d] %f\n ",k,m,new_pop[k].variables[m]);
                m++;
         }


         sum_prb=0.0;
         // normalization
         for(n=1; n <= num_outputs; n++){
                new_pop[k].variables[ini+n]=new_pop[k].variables[ini+n]/sum;
                sum_prb= sum_prb + new_pop[k].variables[ini+n];
                printf(" pop. %d c[%d][%d] %f\n",k,nn,n,new_pop[k].variables[ini+n]);

         }
         printf(" sum. prb C %d %f\n\n",nn,sum_prb);

        }

        printf("Individual %d ",k);
    printf("Num. variables %d num.var used %d\n",num_variables,m-3);

  }



}

void transform_state_machine_hmm(char *path, int num_individuals){

  int num_bits_states,num_bits_output,num_bits_inputs;
  int num_bits_individuals;
  FILE *fp;
  char file_name[300],file_individual[300],file_name_old[300];
  char file_name_hmm[300];
  int i,j,k;
  cromosome_potential new_pop[MAX_NUMBER_IND];
  cromosome_binary old_pop[MAX_NUMBER_IND];
  int num_best=NUM_BEST;
  float num_best_factor;
  int num_variables;
  int num_bits_variables=3;
  int num_states=16;
  int num_outputs=8;
  int num_symbols=256;


  // It gets the argument data

  if (num_individuals> MAX_NUMBER_IND) {
    printf("Increase MAX_NUMBER_IND in file genetics.h \n");
    exit(1);
  }


  strcpy(file_name,path);
  strcpy(file_name_old,path);
  strcpy(file_name_hmm,path);
  strcat(file_name,"avoid_hmm");
  strcat(file_name_old,"old_avoid_hmm");
  strcat(file_name_hmm,"avoid_hmm");

  num_variables= num_states * (1 + num_states + num_symbols + num_symbols*num_outputs);
  num_bits_individuals = num_bits_variables * num_variables;
  num_variables = num_variables + 3;
  printf("Num. variables %d num.bits_individuals %d\n",num_variables,num_bits_individuals);

  for(i=0;i< num_individuals;i++){
        old_pop[i].bits =  (int *) alloc_int(num_bits_individuals);
  }

  for(i=0;i< num_individuals;i++){
        new_pop[i].variables =  (float *) alloc_float(num_variables);
  }


  //printf("Read the file with population %s \n",file_name);
  read_pop_binary_transform(file_name,old_pop,num_individuals,num_bits_individuals);

  // it transforms the bits to an structure that the hmm uses  
  transform_pts_hmm(old_pop,new_pop,num_variables,num_bits_variables,num_individuals,
                        num_states,num_outputs,num_symbols);

  for(i=0;i<num_individuals;i++){
        // it saves the new population
        sprintf(file_individual,"%s_%d.dat",file_name_hmm,i);
        fprint_state_output_hmm(i,new_pop,file_individual,num_bits_inputs);
  }


}
void fprint_mdp(int index,int num_variables, cromosome_potential *new_pop, char *file_individual){

  int i,j,k;
  FILE *fp2;
  int m=0;

  if ((fp2=fopen(file_individual,"w"))==NULL){
                printf("Error opening file %s\n",file_individual);
                exit(1);
  }

  printf("%s\n",file_individual);

  // It prints the number of raws and columns of the reward matrix; ths information is in ../utilities/structures.h
  fprintf(fp2,"%d %d \n",ROW_RWD,CLM_RWD);

// It writes the type and the rewards of the MDP celds 
   for(j=1;j<=ROW_RWD;j++){
        for(k=1;k<=CLM_RWD;k++){
                fprintf(fp2,"1 -0.040000 ");
        }
        fprintf(fp2,"\n");
   }


   // It prints the probabilities 
   for(i=1;i<=NUM_MDP_PRB;i++){
    for(j=1;j<=ROW_PRB;j++){
        for(k=1;k<=CLM_PRB;k++){
          fprintf(fp2,"%f ",new_pop[index].variables[m]);
          //printf("%f ",new_pop[index].variables[m]);
    m++;
        }
        fprintf(fp2,"\n");
    }
    fprintf(fp2,"\n");
   }

  fclose(fp2);

}
void transform_mdp_bits(cromosome_binary *old_pop, cromosome_potential *new_pop, int size_table, int num_prb, int num_bits_variables, int num_individuals){

  int i,j;
  int k,l;
  int bits[100];
  float var;
  int m=0;
  //int num_states,num_inputs,num_outputs;
  int vari;
  float sum;
  int ini,n,nn;
  float sum_prb;


  for(k=0;k<num_individuals;k++){

     printf("Individual %d \n",k);

     i=0;
     m=0;

     // it gets the components of the MDP table
     for(nn=1; nn <= size_table; nn++){


      sum=0.0;
      ini=m-1;

      // it gets the probabilities of one table's raw
      for(n=1; n <= num_prb; n++){

                // it gets the probabilities of one element of the raw
                for(j=0;j < num_bits_variables; j++){
                        bits[j]=old_pop[k].bits[i];
                        //printf("%d",bits[j]);
                        i++;
                }

                var=0;
                for(j=num_bits_variables-1,l=0;j >= 1; j--,l++){
                        var=bits[j]*(2 << l)+var;
                }

                var= var  + 0.001;
                sum=sum+var;
                new_pop[k].variables[m]=var;
                //printf(" pop. %d var[%d] %f\n ",k,m,new_pop[k].variables[m]);
                m++;
      }


      sum_prb=0.0;
      // normalization
      for(n=1; n <= num_prb; n++){
                new_pop[k].variables[ini+n]=new_pop[k].variables[ini+n]/sum;
                sum_prb= sum_prb + new_pop[k].variables[ini+n];
                //printf(" pop. %d table[%d][%d] %f\n",k,nn,n,new_pop[k].variables[ini+n]);

      }
      //printf(" sum. prb %d %f\n\n",nn,sum_prb);

    }

    printf("Individual %d ",k);
    printf("Num. variables %d\n",m);

 }


}





//transform mdp path num_individuals num_bits_variables num_bits_fractions num_variables name_pop
//transform_gen mdp /home/savage/data/data_15/ 100 8 7 72 avoid
//void transform_mdp(char *args[],int nargc){
void transform_mdp(char *path, int num_individuals){

  int num_variables,num_bits_fractions;
  int num_bits_individuals,num_bits_variables;
  FILE *fp;
  char file_name[300],file_individual[300],file_name_old[300];
  char file_out[300];
  int i,j,k;
  cromosome_potential new_pop[MAX_NUMBER_IND];
  cromosome_binary old_pop[MAX_NUMBER_IND];
  float mut_factor=0.0,cross_factor=0.0;
  int num_best=NUM_BEST;
  float num_best_factor;


  // It gets the argument data
  printf("PATH %s \n",path);
  if (num_individuals> MAX_NUMBER_IND) {
    printf("Increase MAX_NUMBER_IND in file genetics.h \n");
    exit(1);
  }
  num_bits_variables=3;
  num_bits_fractions=7;
  num_variables=72;
  strcpy(file_name,path);
  strcpy(file_out,path);
  //strcat(file_out,"pot_");
  strcat(file_name,"avoid_mdp");
  strcat(file_out,"avoid_mdp");
  strcpy(file_name_old,path);
  strcat(file_name_old,"old_");
  strcat(file_name_old,"avoid_mdp");



  num_bits_individuals=num_bits_variables*num_variables;

  for(i=0;i< num_individuals;i++){
        old_pop[i].bits =  (int *) alloc_int(num_bits_individuals);
  }

  for(i=0;i< num_individuals;i++){
        new_pop[i].variables =  (float *) alloc_float(num_variables);
  }


  //printf("Read the file with population %s \n",file_name);
  read_pop_binary_transform(file_name,old_pop,num_individuals,num_bits_individuals);

  // it transforms the bits to an structure that the mdp uses  
  transform_mdp_bits(old_pop,new_pop,NUM_MDP_PRB,ROW_PRB*CLM_PRB,num_bits_variables,num_individuals);

  for(i=0;i<num_individuals;i++){
        // it saves the new population
        sprintf(file_individual,"%s_%d.dat",file_out,i);
        fprint_mdp(i,num_variables,new_pop,file_individual);

  }


}
int crossover_binary(cromosome_binary *new_pop, int ind1, int ind2, float cross_factor, int num_bits_individuals){

  float num;
  static cromosome_binary temp;
  int i,j;
  static int flag=0;
  int z1,z2,temp_z;
  int num_bits_cross;
  int cross_type=CROSS_TYPE;
  int tmp;


  if(flag==0){
  temp.bits =  (int *) alloc_int(num_bits_individuals);
  flag=1;
  }


  num_bits_cross=(num_bits_individuals-1)*PERCENTAGE_CROSS;
  if(num_bits_cross<=1)num_bits_cross=2;

  switch(cross_type){
                case 1: 
        z1=(int) generaR(0.0,num_bits_individuals-1);
        z2=(int) generaR(0.0,num_bits_cross-1)+z1;
        if(z2 > num_bits_individuals){
        z2=num_bits_individuals;
      }

      // it crosses the bits
      printf("it cross bits from %d to %d of individuals %d %d\n",z1,z2,ind1,ind2);
      for (i=z1;i<=z2;i++){
              //temp.bits[i] = new_pop[ind1].bits[i];
              tmp = new_pop[ind1].bits[i];
        new_pop[ind1].bits[i]=new_pop[ind2].bits[i];  
        new_pop[ind2].bits[i]=tmp;
      }
                        break;

                case 2: 
      z1=(int) generaR(0.0,num_bits_individuals-1);
      printf("it cross bits from 0 to %d and to the end of individuals %d %d\n",z1,ind1,ind2);
      for (i=0;i<z1;i++){
                                new_pop[ind2].bits[i]=new_pop[ind1].bits[i];
                        }
      for (i=z1;i<=num_bits_individuals;i++){
                                new_pop[ind1].bits[i]=new_pop[ind2].bits[i];
                        }
                        break;
  
                case 3: 
      z1=(int) generaR(0.0,num_bits_individuals-1);
                        z2=(int) generaR(0.0,num_bits_cross-1)+z1;
                        if(z2 > num_bits_individuals){
                                z2=num_bits_individuals;
                        }


      for (i=0;i<=num_bits_individuals;i++){
                                temp.bits[i]=new_pop[ind1].bits[i];
                        }

                        // it crosses the bits
                        printf("it cross bits from %d to %d of individuals %d to %d\n",z1,z2,ind2,ind1);
                        for (i=z1;i<=z2;i++){
                                new_pop[ind1].bits[i]=new_pop[ind2].bits[i];
                        }

      z1=(int) generaR(0.0,num_bits_individuals-1);
                        z2=(int) generaR(0.0,num_bits_cross-1)+z1;
                        if(z2 > num_bits_individuals){
                                z2=num_bits_individuals;
                        }

      // it crosses the bits
                        printf("it cross bits from %d to %d of individuals %d to %d\n",z1,z2,ind1,ind2);
                        for (i=z1;i<=z2;i++){
                                new_pop[ind2].bits[i]=temp.bits[i];
                        }
                        break;

                default:
                        break;
         }  

 

}
int mutation_binary(cromosome_binary *new_pop, int index, float mut_factor, int num_bits_individuals, float porcentage_num_bits_mut){
  int lug;
  float num;
  float m;
  int i,j,k;
  int num_bits_mutation;

  num_bits_mutation=(int)(num_bits_individuals-1)*PERCENTAGE_MUT;
  if(num_bits_mutation==0)num_bits_mutation=1;


  i=(int)(num_bits_mutation*porcentage_num_bits_mut);
  if(i<=1)i=2;

  k= (int) generaR(0.0,i-1);

  for(j=0;j<= k; j++){
  
  i= (int) generaR(0.0,num_bits_individuals-1);
  new_pop[index].bits[i] = 1 - new_pop[index].bits[i];
    printf("mutation individual %d num.mut %d bit %d\n",index,j,i);
  }

}

void new_pop_vasconcelos(cromosome_binary *old_pop, cromosome_binary *new_pop, int num_bits_individuals, int num_individuals,
                                 float mut_factor, float cross_factor,int num_best,float porcentage_num_bits_mut){

  int i,pts,j;
  int i1,i2,i3,i4;
  int ibest;
  int ii;
  float m_mut,m;
  float num;
  int k;
  int upper,l;
  float n_bits_mut;


  // it selects the N individuals from the old generation
  for(k=0;k<num_best;k++){
    printf("It leaves the same individual %d\n",k);
    for (i=0;i<num_bits_individuals;i++){
                new_pop[k].bits[i]=old_pop[k].bits[i];
    //printf("%d ",new_pop[k].bits[i]);
    }
    //printf("\n");
  }
  num_individuals=2*num_individuals-num_best;
  upper= (num_individuals/2) + num_individuals % 2;
  for (i=0,k=1,l=2; i< upper ;i++,k++,l=l+2){

     // it selects two individuals from the old generation
     i1=i;
     i2=num_individuals-i;
     printf("\nGenerating individuals %d %d\n",l,l+1);
     printf("%d it selects the individuals %d %d\n",i,i1,i2);

     for (ii=0;ii<num_bits_individuals;ii++){
                new_pop[l+1].bits[ii]=old_pop[i1].bits[ii];
                new_pop[l].bits[ii]=old_pop[i2].bits[ii];
     }


     // it crosses the individuals
     num= generaR(0.0,1.0);
     if (num<=m){
        printf("it crosses individuals %d %d\n",i1,i2);
        crossover_binary(new_pop,l,l+1,cross_factor,num_bits_individuals);
     }

     // it mutates the individuals
     num= generaR(0.0,1.0);
     if (num<=m_mut) {
        printf("Mut. individual %d m_mut %f num %f \n",l,m_mut,num);
        mutation_binary(new_pop,l, mut_factor,num_bits_individuals,n_bits_mut);
     }

     num= generaR(0.0,1.0);
     if (num<=m_mut) {
        printf("Mut. individual %d m_mut %f num %f \n",l-1,m_mut,num);
        mutation_binary(new_pop,l+1, mut_factor,num_bits_individuals,n_bits_mut);
     }

  }

  if((num_individuals % 2)== 0){
   i1=num_individuals;
   l--;

   printf("\nGenerating last individual %d\n",l);
   for (ii=0;ii<num_bits_individuals;ii++){
                new_pop[l].bits[ii]=old_pop[i1].bits[ii];
   }

  }

}

int main(int argc,char *args[]){ // args: path, num_individuals, behavior
  int num_individuals,num_bits_individuals;
  FILE *fp;
  char file_name[300],file_individual[300],file_name_old[300];
  int i,j,k;
  cromosome_binary new_pop[MAX_NUMBER_IND],old_pop[MAX_NUMBER_IND];
  int num_best=NUM_BEST;
  char file_population[300];
  float mut_factor=0.6;
  float cross_factor=0.9;
  int num_best_factor=0.8;
  float porcentage_num_bits_mut=0.2;

  char path[300]; //  ../../../src/simulator/src/genetic_behaviors/data/

  char behavior[10];
  //leer args path num_individuals behavior

  strcpy(path,args[1]);
  num_individuals = atoi(args[2]);
  strcpy(behavior,args[3]);
  if(strcmp(behavior,"fsm")==0) num_bits_individuals=28672;
  else if(strcmp(behavior,"hmm")==0) num_bits_individuals=297088;
  else if(strcmp(behavior,"mdp")==0) num_bits_individuals=576;
  //num_individuals=num_individuals*2; 
  sprintf(file_name,"%savoid_%s",path,behavior);
  sprintf(file_name_old,"%savoid_%s_old",path,behavior);
  for(i=0;i< num_individuals+2;i++){
    old_pop[i].bits =  (int *) alloc_int(num_bits_individuals);
    new_pop[i].bits =  (int *) alloc_int(num_bits_individuals);
  }
  if(strcmp(behavior,"fsm")==0) transform_state_machine(path, num_individuals,num_bits_individuals);
  else if(strcmp(behavior,"hmm")==0) transform_state_machine_hmm(path, num_individuals);
  else if(strcmp(behavior,"mdp")==0) transform_mdp(path, num_individuals);
  else printf("Behavior not implemented");
  //printf("Read the file with population %s \n",file_name);
  read_pop_binary(path,behavior,old_pop,num_individuals,num_bits_individuals);
   
  // it sorts the population according to its fitness
  sort(num_individuals,old_pop,num_bits_individuals);
  // for(i=0;i<num_individuals;i++){
  //   sprintf(file_population,"avoid_%s_%i"behavior,num_individuals);
  // 
  // }
  new_pop_vasconcelos(old_pop,new_pop,num_bits_individuals,num_individuals/2,mut_factor,cross_factor,num_best,porcentage_num_bits_mut);


  // it saves the old population
  sprintf(file_population,"%s.dat",file_name_old);
  printf("Old population %s\n",file_population);
  if ((fp=fopen(file_population,"w"))==NULL){
                printf("Error opening file %s\n",file_population);
                exit(1);
  }
  
  for(i=0;i<num_individuals;i++){
        for(k=0; k < num_bits_individuals;k++){
                //printf("%d ",old_pop[i].bits[k]);
                fprintf(fp,"%d ",old_pop[i].bits[k]);
        }
        fprintf(fp,"\n");
        //printf("\n");
 }

 fclose(fp);

 // it saves the new population
 sprintf(file_population,"%s.dat",file_name);
 printf("New population %s\n",file_population);
 if ((fp=fopen(file_population,"w"))==NULL){
                printf("Error opening file %s\n",file_population);
                exit(1);
 }

 for(i=0;i<num_individuals;i++){
        for(k=0; k < num_bits_individuals;k++){
                //printf("%d ",new_pop[i].bits[k]);
                fprintf(fp,"%d ",new_pop[i].bits[k]);
  }

        fprintf(fp,"\n");
        //printf("\n");
 }

 fclose(fp);

 // It writes the fitness file of the old population
 sprintf(file_individual,"%sfitness_generation.dat",path);
 printf("Fitness file %s\n",file_individual); 
 if ((fp=fopen(file_individual,"w"))==NULL){
                printf("Error opening file %s\n",file_individual);
                exit(1);
 }

 //printf("\n\npop %s\n",file_individual);
 for(i=0;i<num_individuals;i++){
    fprintf(fp,"%s_%d.dat %f\n",file_name_old,i,old_pop[i].fitness);
 }

 fclose(fp);
  if(strcmp(behavior,"fsm")==0) transform_state_machine(path, num_individuals,num_bits_individuals);
  else if(strcmp(behavior,"hmm")==0) transform_state_machine_hmm(path, num_individuals);
  else if(strcmp(behavior,"mdp")==0) transform_mdp(path, num_individuals);
  else printf("Behavior not implemented");
 

}  



