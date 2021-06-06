/********************************************************
*	get_hmm_vision.c				*
*							*
*							*
*		University of Mexico U.N.A.M.		*
*			Jesus Savage			*
*							*
*				4/17/2009		*
*				4/17/2009		*
*********************************************************
*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "/home/savage/vision/pattern_recognition/include_vision/basics_vqs_vision.h"
#include "/home/savage/vision/pattern_recognition/vq_vision/new_allocate.h"
#include "hmm_vision_lib.h"



#define SIZE_IMAGE_VECTORS 5000
#define ERROR .000001

int num_centroids_sonar=NUM_CENTROIDS_SONAR;
extern int *alloc_vector_int();



/* it gets line inputs */
int get_inputs(int argc, char **argv, char *object,int *show,int *num_states,char *path, int *size_vq, char *vq_file)
{
 int i,reestimation=0;
 char  data[100];


 strcpy(object,"floor");
 strcpy(vq_file,"all_rgb");
 *show=0;
 *num_states=NUM_STATES;
 strcpy(path,"/home/savage/observations/");
 *size_vq=32;



  /* reads input parameters */
  if (argc>1) {
        for(i=1;i<argc;i++){
          if (!strcmp(argv[i],"-h")){
                show_help("get_hmm_vision");
                exit(1);
          }
          if (!strcmp(argv[i],"-s")){
                *show=1;
          }
          if (!strcmp(argv[i],"-f")){
                strcpy(object,argv[i+1]);
          }
          if (!strcmp(argv[i],"-p")){
                strcpy(path,argv[i+1]);
          }
          if (!strcmp(argv[i],"-q")){
                strcpy(vq_file,argv[i+1]);
          }
          if (!strcmp(argv[i],"-r")){
                reestimation=1;
          }
	  if (!strcmp(argv[i],"-v")){
                *size_vq=atoi(argv[i+1]);
	  }
	  if (!strcmp(argv[i],"-n")){
                *num_states=atoi(argv[i+1]);
				if(*num_states > NUM_STATES){
					printf("\n Increase constant NUM_STATES >= %d\n",*num_states);
					exit(1);
				}
          }

        }
  }
  else{
 	printf("\n Using default values");
  }
  
  printf("\n get_hmm_vision -f %s -n %d -p %s -v %d -q %s\n\n",object,*num_states,path,*size_vq,vq_file);


 return(reestimation);

}


// It updates the hmm probabilities using the Viterbi algorithm
int update_hmm_viterbi(char *object,int num_images,int *num_vectors,int **obs, int num_states, char *path, int num_symbols)
{
 char output_file[250];
 int i,j,k,l,t;
 float tmp,lim,average;
 FILE *fp2,*fp1;
 int num_obs[NUM_STATES+1][NUM_MAX_CENTROIDS+1];
 int num_nds,ret_nodes[256];
 int num_states_obs[NUM_STATES+1];
 int state_obs[NUM_STATES+1][NUM_STATES+1];
 int t1;
 int inc,init,end;
 float **a,*pis,**b;
 double vit_prob[1024];
 int *q[256];
 int *vect,**vectk;
 int state[NUM_STATES+1][NUM_STATES+1];
 double p;




  // It allocates the matrices used for the hmm evaluation

  if((a =(float **)alloc_matrix(num_states+1, num_states+1 )) == 0){
         printf("\n Memory allocation error ");
         exit(1);
  }

  if((b =(float **)alloc_matrix(num_states+1,num_symbols+1)) == 0){
         printf("\n Memory allocation error ");
         exit(1);
  }

  if((pis =(float *)alloc_vector(num_states+1)) == 0){
         printf("\n Memory allocation error ");
         exit(1);
  }

  if((vect =(int *)alloc_vector_int(num_states+1)) == 0){
         printf("\n Memory allocation error ");
         exit(1);
  }

  if((vectk=(int **)alloc_matrix_int(num_states+1,num_symbols+1))==0){
         printf("\n Memory allocation error ");
         exit(1);
  }



 // it allocates the states vectors
 for(i=1;i<num_images;i++)
 if((q[i] = (int *) alloc_vector_int((num_vectors[i]+1))) == 0){
                printf("\n Memory allocation error: seq_states %d %d",i,num_vectors[i]);
                exit(0);
 }


 // it initializes some variables
 for(i=1;i<=num_states;i++){
        vect[i]=0;
        for(j=1;j<=num_states;j++){
                state[i][j]=0;
        }
        for(j=1;j<=num_symbols;j++){
                        vectk[i][j]=0;
        }

 }



 //It reads the HMM model 
 read_hmm_ais_bis_pis(object,a,pis,b,num_states,num_symbols,path);

 p=0;
 for(i=1;i<num_images;i++){

        /* It calculates the best sequence of states using the Viterbi algorithm
         * and the probability of a hmm given a symbol vector */
        vit_prob[i]=viterbi(obs[i],a,b,pis,num_vectors[i],num_states,q[i]);
        vit_prob[i]=exp(vit_prob[i]/10.);
        //printf("Viterbi probability %d %e\n",i,vit_prob[i]);
	p=p+vit_prob[i];

 	for(t=1;t<num_vectors[i];t++){
                state[q[i][t]][q[i][t+1]]++;
                vect[q[i][t]]++;
                vectk[q[i][t]][obs[i][t]]++;
                //printf("state[%d][%d] %d obs[%d][%d] %d\n",q[i][t],q[i][t+1],state[q[i][t]][q[i][t+1]],i,t,obs[i][t]);
        }
 }

 
 printf("total Viterbi probability %e\n",p/(num_images-1));

 /* It updates the ais and bis */
 for(i=1;i<=num_states;i++){
                for(j=1;j<=num_states;j++){
                        if(vect[i] != 0)
                        a[i][j]=(float)(state[i][j]/(float)vect[i]);
                        else a[i][j]=0.;
                        //printf("a[%d][%d] %f\n",i,j,a[i][j]);
                }
                for(j=1;j<=num_symbols;j++){
                        if(vect[i] != 0)
                        	b[i][j]=(float)(vectk[i][j]/(float)vect[i]);
                        else 	b[i][j]=0.;
                	//printf("\n b[%d][%d] %f",i,j,b[i][j]);
                }
 }






 // It checks for probabilities values less than NULL_PROB and fixed it
 for(k=1;k<=num_states;k++){

        lim=0;
        for(l=1;l<=num_states;l++){
              if(a[k][l]<NULL_PROB){
                  a[k][l]=NULL_PROB;
                  lim=lim+1;
		  //printf("%d %d lim %f\n",k,l,lim);
	          //printf("a[%d][%d] %f\n",k,l,a[k][l]);
              }
        }
        /*it fixes the probabilities in order that the total probability is 1 */
        if(lim != 0){
          if(lim == num_states){
                printf("\n Please retrain the object several times \n");
                exit(0);
          }


          for(l=1;l<= num_states;l++)
            if((a[k][l]!=NULL_PROB)&&(a[k][l]>(lim*NULL_PROB)/(num_states-lim)))
                a[k][l]=a[k][l]-(lim*NULL_PROB)/(float)(num_states-lim);

        }

   }



 for(k=1;k<=num_states;k++){
        lim=0;
        for(l=1;l<=num_symbols;l++){
               if(b[k][l]<=NULL_PROB){
                  b[k][l]=(float) NULL_PROB;
                  lim++;
                }
        }

        //it fixes the probabilities in order that the total probability is 1
        if(lim != 0){
          for(l=1;l<= num_symbols;l++)
            if((b[k][l] > NULL_PROB)
              &(b[k][l] > ((lim*NULL_PROB)/(float)(num_symbols-lim)) )){

               b[k][l]=b[k][l]-(lim*(float)NULL_PROB)/(float)(num_symbols-lim);

            }

        }
 }



 // It saves the transition and initial probabilities 
 sprintf(output_file,"%s%s_%d.prb",path,object,num_symbols);
 fp2=fopen(output_file,"w");
 if(fp2==0){
             printf("\n Can not open %s ",output_file);
             return(0);
 }



 for(j=1;j<=num_states;j++)fprintf(fp2,"%f ",pis[j]);
 fprintf(fp2,"\n");
 average=0;
 for(j=1;j<=num_states;j++){
        //printf(" pi[%d] %f\n",j,pis[j]);
        average+=pis[j];
 }
 //printf("sum pis %f\n",average);

 for(j=1;j<=num_states;j++){
        average=0;
        for(k=1;k<=num_states;k++){
                fprintf(fp2,"%f ",a[j][k]);
                //printf("\n a[%d][%d] %f",j,k,a[j][k]);

                average+=a[j][k];
        }
        fprintf(fp2,"\n");
        //printf("\nsum ais[%d] %f\n",j,average);
 }
 //printf("\n");

 for(j=1;j<=num_states;j++){
                average=0;
                for(k=1;k<=num_symbols;k++){
                     //printf("b[%d][%d] %f\n",j,k,b[j][k]);
                     average+=b[j][k];
                     fprintf(fp2,"%f ",b[j][k]);
                }
                //printf("\n");
                //printf("sum bis[%d] %f\n",j,average);
                fprintf(fp2,"\n");
 }

 fclose(fp2);

 //printf("NEW HMM FILE: %s\n",output_file);


}




// It creates the initial probabilities
int create_initial_hmm(char *object,int num_images,int *size_images,int **obs, int num_states, char *path, int num_symbols)
{
 char output_file[250];
 //int *vect,***vectk,*q;
 float a[NUM_STATES+1][NUM_STATES+1], b[NUM_STATES+1][NUM_MAX_CENTROIDS+1];
 float pi[NUM_STATES+1];
 int i,j,k,l;
 float tmp,lim,average;
 FILE *fp2,*fp1;
 int num_vectors[NUM_STATES+1][NUM_STATES+1];
 int num_obs[NUM_STATES+1][NUM_MAX_CENTROIDS+1];
 int num_nds,ret_nodes[256];
 int num_states_obs[NUM_STATES+1];
 int state_obs[NUM_STATES+1][NUM_STATES+1];
 int t1;
 int inc,init,end;


 /* It initializes vectors and matrices */
 for(i=1;i<=num_states;i++){
        pi[i]= (float) (1./num_states);
        for(j=1;j<=num_states;j++){
                a[i][j]=(float)NULL_PROB;
		//printf("\n a[%d][%d] %f",i,j,a[i][j]);
  		// it assigns the states associated to the section of the image
		state_obs[i][j]=j;
        }
        for(j=1;j<=NUM_CENTROIDS_SONAR;j++){
                b[i][j]=(float)NULL_PROB;
                num_obs[i][j]=0;
		//printf("\n b[%d][%d] %f",i,j,b[i][j]);
        }
	num_states_obs[i]=num_states;
 }



  // it assigns the states associated to the sections of the images
/*
 num_states_obs[1]=4;
 state_obs[1][1]=1;
 state_obs[1][2]=2;
 state_obs[1][3]=4;
 state_obs[1][4]=5;
*/


 num_states_obs[1]=2;
 state_obs[1][1]=1;
 state_obs[1][2]=2;

/*
 num_states_obs[2]=6;
 state_obs[2][1]=1;
 state_obs[2][2]=2;
 state_obs[2][3]=3;
 state_obs[2][4]=4;
 state_obs[2][5]=5;
 state_obs[2][6]=6;
*/

 num_states_obs[2]=3;
 state_obs[2][1]=1;
 state_obs[2][2]=2;
 state_obs[2][3]=3;

/*
 num_states_obs[3]=4;
 state_obs[3][1]=2;
 state_obs[3][2]=3;
 state_obs[3][3]=5;
 state_obs[3][4]=6;
*/


 num_states_obs[3]=3;
 state_obs[3][1]=2;
 state_obs[3][2]=3;
 state_obs[3][3]=4;


/*
 num_states_obs[4]=6;
 state_obs[4][1]=1;
 state_obs[4][2]=2;
 state_obs[4][3]=4;
 state_obs[4][4]=5;
 state_obs[4][5]=7;
 state_obs[4][6]=8;
*/


 num_states_obs[4]=3;
 state_obs[4][1]=3;
 state_obs[4][2]=4;
 state_obs[4][3]=5;


/*
 num_states_obs[5]=9;
 state_obs[5][1]=1;
 state_obs[5][2]=2;
 state_obs[5][3]=3;
 state_obs[5][4]=4;
 state_obs[5][5]=5;
 state_obs[5][6]=6;
 state_obs[5][7]=7;
 state_obs[5][8]=8;
 state_obs[5][9]=9;
*/


 num_states_obs[5]=3;
 state_obs[5][1]=4;
 state_obs[5][2]=5;
 state_obs[5][3]=6;


/*
 num_states_obs[6]=4;
 state_obs[6][1]=2;
 state_obs[6][2]=3;
 state_obs[6][3]=5;
 state_obs[6][4]=6;
 state_obs[6][5]=8;
 state_obs[6][6]=9;
*/


 num_states_obs[6]=3;
 state_obs[6][1]=5;
 state_obs[6][2]=6;
 state_obs[6][3]=7;

/*
 num_states_obs[7]=4;
 state_obs[7][1]=4;
 state_obs[7][2]=5;
 state_obs[7][3]=7;
 state_obs[7][4]=8;
*/



 num_states_obs[7]=3;
 state_obs[7][1]=6;
 state_obs[7][2]=7;
 state_obs[7][3]=8;



/*
 num_states_obs[8]=6;
 state_obs[8][1]=4;
 state_obs[8][2]=5;
 state_obs[8][3]=6;
 state_obs[8][4]=7;
 state_obs[8][5]=8;
 state_obs[8][6]=9;
*/


 num_states_obs[8]=3;
 state_obs[8][1]=7;
 state_obs[8][2]=8;
 state_obs[8][3]=9;



/*
 num_states_obs[9]=4;
 state_obs[9][1]=5;
 state_obs[9][2]=6;
 state_obs[9][3]=8;
 state_obs[9][4]=9;
*/


 num_states_obs[9]=2;
 state_obs[9][1]=8;
 state_obs[9][2]=9;




 for(i=1;i<=num_states;i++){

 	for(j=1;j<=num_states_obs[i];j++){
 	//for(j=1;j<=num_states;j++){
		a[i][state_obs[i][j]]= (float)1.0/(num_states_obs[i]) - (num_states-(num_states_obs[i]))*NULL_PROB/num_states_obs[i];

		//a[i][j]=1.0/num_states;

	}

 }



 // The observation are assigned to the states
 for(i=1;i<num_images;i++){
 	
	inc=size_images[i]/num_states;

	init=1;
	end=inc;

	for(t1=1;t1<=num_states;t1++){
		
		for(k=1;k<=num_states_obs[t1];k++){
			for(j=init;j<=end;j++){
				num_obs[state_obs[t1][k]][obs[i][j]]++;

			}
		}

		init=end+1;
		end=end+inc;

	}

 }



 for(i=1;i<=num_states;i++){

	tmp=0;
        for(j=1;j<=num_symbols;j++){
			tmp+=num_obs[i][j];
        }
	for(j=1;j<=num_symbols;j++){
		if(tmp==0)tmp=1;
                b[i][j]=(float)(num_obs[i][j]/tmp);
		//printf("\n b[%d][%d] %f",i,j,b[i][j]);
        }
 }


 // It checks for probabilities values less than NULL_PROB and fixed it

 for(k=1;k<=num_states;k++){
        lim=0;
        for(l=1;l<=num_symbols;l++){
               if(b[k][l]<=NULL_PROB){
                  b[k][l]=(float) NULL_PROB;
                  lim++;
                }
        }

        //it fixes the probabilities in order that the total probability is 1 
        if(lim != 0){
          for(l=1;l<= num_symbols;l++)
            if((b[k][l] > NULL_PROB)
              &(b[k][l] > ((lim*NULL_PROB)/(float)(num_symbols-lim)) )){

               b[k][l]=b[k][l]-(lim*(float)NULL_PROB)/(float)(num_symbols-lim);

            }

        }
 }

 /* It saves the transition and initial probabilities */
 sprintf(output_file,"%s%s_%d.prb",path,object,num_symbols);
 fp2=fopen(output_file,"w");
 if(fp2==0){
             printf("\n Can not open %s ",output_file);
             return(0);
 }



 for(j=1;j<=num_states;j++)fprintf(fp2,"%f ",pi[j]);
 fprintf(fp2,"\n");
 average=0;
 for(j=1;j<=num_states;j++){
        printf(" pi[%d] %f\n",j,pi[j]);
        average+=pi[j];
 }
 printf("sum pis %f\n",average);

 for(j=1;j<=num_states;j++){
        average=0;
        for(k=1;k<=num_states;k++){
                fprintf(fp2,"%f ",a[j][k]);
                printf("\n a[%d][%d] %f",j,k,a[j][k]);

                average+=a[j][k];
        }
        fprintf(fp2,"\n");
        printf("\nsum ais[%d] %f\n",j,average);
 }
 printf("\n");

 for(j=1;j<=num_states;j++){
                average=0;
                for(k=1;k<=num_symbols;k++){
                     printf("b[%d][%d] %f\n",j,k,b[j][k]);
                     average+=b[j][k];
                     fprintf(fp2,"%f ",b[j][k]);
                }
                printf("\n");
                printf("sum bis[%d] %f\n",j,average);
                fprintf(fp2,"\n");
 }

 fclose(fp2);

 //printf("HMM FILE: %s\n",output_file);

 sprintf(output_file,"%sretrain_nodes.dat",path,object);
 fp1=fopen(output_file,"w");
 if(fp1==0){
             printf("\n Can not open %s ",output_file);
             return(0);
 }


 for(j=1;j<num_nds;j++)fprintf(fp1,"%d.0\n",ret_nodes[j]);
 fclose(fp1);


}




main(int argc,char *argv[])
{
 int num_vectors;
 int update,show_prob;
 char object[128];
 Raw *vectors,*total_vectors[100];
 Centroid *centroids_vision;
 int *observations[100], num_states,flg_sonar;
 char path[300];
 char data_file[300];
 int num_file,flag,num_total_vectors[100];
 int size_vectors;
 int i,j,k,l;
 int size_vq;
 int num_centroids_vision;
 char vq_file[300];
 int state[NUM_STATES+1][NUM_STATES+1];
 double bigger,prob[NUM_MAX_CENTROIDS],p;
 double vit_prob[NUM_MAX_CENTROIDS];
 float **ais,*pis,**bis;
 double p_old,error;


 /* it gets line inputs */
 update=get_inputs(argc, argv,object,&show_prob,&num_states,path,&size_vq,vq_file);


 /* it gets the number of vectors and their size*/
 num_file=1;
 flag=1;
 while(flag){

                sprintf(data_file,"%simages_training_%s_%d.raw",path,object,num_file);
                num_vectors=get_size_image_vectors(data_file,&size_vectors);
                printf("%s num. vectors %d size vectors %d\n",data_file,num_vectors, size_vectors);

                if(num_vectors== 0){
                        if(num_file==1){
                                printf("\n Can not open %s \n",data_file);
                                exit(0);
                        }
                        flag=0;
                }
                else{
			num_total_vectors[num_file]=num_vectors;
                        num_file++;
                }
 }



 /* it allocates raw vectors space */
 if((vectors = (Raw *) malloc( (SIZE_IMAGE_VECTORS+1) * sizeof(Raw))) == NULL){
                fprintf(stdout,"Allocation error: Raw vectors");
                return(0);
 }

 for(k=1;k<num_file;k++){
 	if((total_vectors[k] = (Raw *) malloc( ( num_total_vectors[k]+1) * sizeof(Raw))) == NULL){
                fprintf(stdout,"Allocation error: Raw vectors %d\n",k);
                return(0);
 	}
 }


  // It allocates the matrices used for the hmm evaluation

  if((ais =(float **)alloc_matrix(num_states+1, num_states+1 )) == 0){
         printf("\n Memory allocation error ");
         exit(1);
  }

  if((bis =(float **)alloc_matrix(num_states+1,size_vq+1)) == 0){
         printf("\n Memory allocation error ");
         exit(1);
  }

  if((pis =(float *)alloc_vector(num_states+1)) == 0){
         printf("\n Memory allocation error ");
         exit(1);
  }


/* it reads the image vectors */
 for(k=1;k<num_file;k++){

                sprintf(data_file,"%simages_training_%s_%d.raw",path,object,k);
                num_vectors= read_image_vectors(data_file,vectors);
                printf("%s num. vectors %d\n",data_file,num_vectors);

                for(j=1;j<=num_vectors;j++){
                        for(l=1;l<=size_vectors;l++){
                                total_vectors[k][j].sensors[l]=vectors[j].sensors[l];
                                //printf("total_vectors[%d][%d].sensors[%d] %f\n",k,j,l,total_vectors[k][j].sensors[l]);
                        }

                }

 }


 // it reads the centroids 
 // it allocates centroids space
 if((centroids_vision = (Centroid *)
            	malloc( (size_vq+1) * sizeof(Centroid))) == NULL){
                fprintf(stdout,"Allocation error: centroids");
                exit(0);
 }

 num_centroids_vision= read_vision_centroids(vq_file,centroids_vision,size_vectors,size_vq,path); 


 // It quantizes the data 
 for(k=1;k<num_file;k++)
 if((observations[k] = (int *) alloc_vector_int((num_total_vectors[k]+1)) ) == 0){
         printf("\n Memory allocation error: observations ");
         exit(0);
 }


 for(k=1;k<num_file;k++){
 	quantize_vectors(centroids_vision,num_total_vectors[k],total_vectors[k],observations[k],num_centroids_vision,size_vq);
 }

 // It finds if the initial probabilities need to be created 
 if(update==0){
       create_initial_hmm(object,num_file,num_total_vectors,observations,num_states,path,num_centroids_vision);
 }

 // It reads the initial HMM model
 read_hmm_ais_bis_pis(object,ais,pis,bis,num_states,size_vq,path);

 p=0;
 // It calculates the probabilities of each of the observation vectors
 for(i=1;i<num_file;i++){

        prob[i]=get_hmm_prob(observations[i],ais,bis,pis,num_vectors,num_states);
        // The result is divided by 10 in order that the result fits the variable size
        prob[i]=exp(prob[i]/10.);
        printf("initial object %s probability %e\n",object,prob[i]);
	p=prob[i]+p;
 }

 p=p/(num_file-1);
 printf("initial total probability %e\n",p);
 p_old=p;
 flag=1;
 k=1;

 while(flag){
 //for(k=1;k<60;k++){
 	// It updates the HMM using the Viterbi algorithm 
 	update_hmm_viterbi(object,num_file,num_total_vectors,observations,num_states,path,num_centroids_vision);

 	// It reads the new HMM model
 	read_hmm_ais_bis_pis(object,ais,pis,bis,num_states,size_vq,path);


 	// It calculates the probabilities of each of the observation vectors
 	for(i=1;i<num_file;i++){

 		prob[i]=get_hmm_prob(observations[i],ais,bis,pis,num_vectors,num_states);
   		// The result is divided by 10 in order that the result fits the variable size
        	prob[i]=exp(prob[i]/10.);
        	//printf("object %s probability %e\n",object,prob[i]);
		p=prob[i]+p;

 	}

 	p=p/(num_file-1);
	error=abs1(p-p_old);
 	printf("%d new total probability %e old prob %e error %e\n",k,p,p_old,error);
	p_old=p;
	k++;
	if(error == 0) flag=0;
	if(k> 100) flag=0;
	if(k> 100) flag=0;
 
 }


 printf("total output file %s%s_%d.prb\n",path,object,num_centroids_vision);

}



