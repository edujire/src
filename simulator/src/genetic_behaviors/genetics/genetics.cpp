
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
  for (i=0,k=1,l=num_individuals; i< upper ;i++,k++,l=l+2){

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
  if(strcmp(behavior,"fsm")) num_bits_individuals=28672;
  //TODO: poner casos para los otros comportamientos
  //num_individuals=num_individuals*2; 
  sprintf(file_name,"%savoid_%s",path,behavior);
  sprintf(file_name_old,"%savoid_%s_old",path,behavior);
  for(i=0;i< num_individuals+2;i++){
    old_pop[i].bits =  (int *) alloc_int(num_bits_individuals);
    new_pop[i].bits =  (int *) alloc_int(num_bits_individuals);
  }
 
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
 transform_state_machine(path, num_individuals,num_bits_individuals);

}  



