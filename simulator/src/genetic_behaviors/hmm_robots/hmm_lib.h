/********************************
*				*
*	 hmm_lib.h		*
*	 Basic HMM libraries	*
*				*
*		Jesus Savage	*
*		FI-UNAM		*
*		4-24-2009	*
*		5-5-2013	*
*				*
*				*
*********************************/

//#include "new_allocate.h"

#define NUM_STATES 32
#define NUM_BITS_DESTINATION 3
#define NUM_MAX_CENTROIDS 1024
#define NUM_MAX_OBSERVATIONS 500
//#define PATH "../data/"
//#define NUM_MAX_SENSORS 4097
#define REAL float
#define NUM_MAX_BEHAVIORS 100
#define NULL_PROB 0.0001


/*
typedef struct _Centroid{
        int num;
        float x,y,theta,sensors[NUM_MAX_SENSORS];
} Centroid;
*/


struct hmm_database {
        char objects_file[300];
        double bigger, prob[NUM_MAX_CENTROIDS], p;
        double vit_prob[NUM_MAX_CENTROIDS];
        int index;
        int index_vit ;
        double bigger_vit;
        float **ais, *pis, **bis, **cis;
        int num_bits_inputs;
        int num_states;
	int num_obs_symbols;
	int num_out_symbols;
	int num_vectors;
        int  *seq_states;
        int *observations;
};




struct vector_quantizer {
        char  in_file[300];
        int num_vectors;
        Centroid *centroids;
        Centroid *centroids_vision;
        int size_vectors; //=3
        Raw *vectors;
        char path[300];
        char data_file[300];
        int size_vq;
        int num_centroids_vision;
        char vq_file[300];
};


int **alloc_matrix_int(int row, int col)
{
    int i;
    int **m;

    m = (int **) malloc((unsigned) row*sizeof(int *));
    for (i=0; i<row; i++)
        m[i] = (int *) malloc((unsigned) col*sizeof(int));
    return m;
}


double *alloc_vector_double(int len)
{
    return (double *) malloc((unsigned) len*sizeof(double));
}



/*REAL **alloc_matrix(int row,int col)
{
    int i;
    REAL **m;

    m = (REAL **) malloc((unsigned) row*sizeof(REAL *));
    for (i=0; i<row; i++)
        m[i] = (REAL *) malloc((unsigned) col*sizeof(REAL));
    return m;
}
*/


REAL *alloc_vector(int len)
{
    return (REAL *) malloc((unsigned) len*sizeof(REAL));

}

int *alloc_vector_int(int len)
{
    return (int *) malloc((unsigned) len*sizeof(int));
}

double **alloc_matrix_double(int row,int col)
{
    int i;
    double **m;

    m = (double **) malloc((unsigned) row*sizeof(double *));
    for (i=0; i<row; i++)
        m[i] = (double *) malloc((unsigned) col*sizeof(double));
    return m;
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

void free_vector_double(double *v)
{
    free((double *) v);
}

void free_vector_int(int *v)
{
    free( (int *) v);
}



void vq_initialize(struct vector_quantizer *vq) {
        if ((vq->centroids_vision = (Centroid *)
                malloc( (vq->size_vq + 1) * sizeof(Centroid))) == NULL) {
                fprintf(stdout, "Allocation error: centroids");
                exit(0);
        }
        vq->size_vectors = 3;
        //vq->num_centroids_vision = read_vision_centroids(vq->vq_file, vq->centroids_vision, vq->size_vectors, vq->size_vq, vq->path);
        vq->size_vq=5;

}




void vq_allocate_vectors(struct vector_quantizer *vq) {
        if ((vq->vectors = (Raw *) malloc( (vq->num_vectors + 1) * sizeof(Raw))) == NULL) {
                fprintf(stdout, "Allocation error: Raw vectors");
                exit(0);
        }
}



// it initializes the hmm
void hmm_initialize(struct hmm_database *hmm) {
        int i,j;

        hmm->index = 1;
        hmm->index_vit = -1;


	// It allocates the matrices used for the hmm evaluation
        if ((hmm->ais = (float **)alloc_matrix(hmm->num_states + 1, hmm->num_states + 1 )) == 0) {
                        printf("\n Memory allocation error ");
                        exit(1);
        }
        if ((hmm->bis = (float **)alloc_matrix(hmm->num_states + 1, hmm->num_obs_symbols + 1)) == 0) {
                        printf("\n Memory allocation error ");
                        exit(1);
        }
        if ((hmm->cis = (float **)alloc_matrix(hmm->num_states*hmm->num_obs_symbols + 1, hmm->num_out_symbols + 1)) == 0) {
                        printf("\n Memory allocation error ");
                        exit(1);
        }
        if ((hmm->pis = (float *)alloc_vector(hmm->num_states + 1)) == 0) {
                        printf("\n Memory allocation error ");
                        exit(1);
        }

        if ((hmm->observations = (int *) alloc_vector_int((hmm->num_vectors + 1)) ) == 0) {
                printf("\n Memory allocation error: observations ");
                exit(0);
        }

	// it allocates the states vectors
        if ((hmm->seq_states = (int *) alloc_vector_int((hmm->num_vectors + 1))) == 0) {
                        printf("\n Memory allocation error: observations ");
                        exit(0);
        }

}
     
/* Function that that calculate the first state from the pis */
int calculate_hmm_first_state(struct hmm_database hmm)
{
 int i,j;
 float rand;
 float tmp;


 // it generates a Uniform random variable between 0 and 1
 rand=generaR(0.0,1.0);
#ifdef DEBUG
 printf(" random %f\n",rand);
#endif

 tmp=hmm.pis[1];

 for(j=1;j<=hmm.num_states;j++){
        if(rand <= tmp){
		i=j;
		j=hmm.num_states+1;
	} else{
		tmp= tmp + hmm.pis[j+1];
		i=j;
	}
   }

#ifdef DEBUG
   printf(" Initial HMM state %d tmp %f\n",i,tmp);
#endif

   return i;

}


/* Function that that calculate the first state from the pis */
int calculate_region(float *prob, int num_regions)
{
 int i,j;
 float rand;
 float tmp;


 // it generates a Uniform random variable between 0 and 1
 rand=generaR(0.0,1.0);
 //printf(" random %f\n",rand);


 tmp=prob[1];

 for(j=1;j<=num_regions;j++){
        if(rand <= tmp){
                i=j;
                j=num_regions+1;
        } else{
                tmp= tmp + prob[j+1];
                i=j;
        }
   }

   //printf(" fall region %d prb %f\n",i,tmp);

   return i;

}





/* Function that calculates the HMM output from the C matrix */
int calculate_hmm_output(struct hmm_database hmm,int state, int obs, int flg_inputs)
{
 int i,j;
 float rand;
 float tmp,threshold=0.0001;
 int index;
 //char outputs[10][15] = { "STOP","FORWARD","BACKWARD","LEFT","RIGHT"};

 if(flg_inputs==0) index= state;
 else index= (state-1) * hmm.num_obs_symbols + (obs + 1);

#ifdef DEBUG
 printf("state %d obs %d\n",state,obs);
 printf("index %d \n",index);
#endif


 // it generates a Uniform random variable between 0 and 1
 rand=generaR(0.0,1.0);
#ifdef DEBUG
 printf("output random %f index %d\n",rand,index);
#endif


 tmp=hmm.cis[index][1];

#ifdef DEBUG
 printf(" HMM 1 sum %f\n",tmp);
#endif

 for(j=1;j<=hmm.num_out_symbols;j++){
        if(rand <= tmp){
                i=j;
                j=hmm.num_out_symbols+1;
        } else{
                tmp= tmp + hmm.cis[index][j+1];
#ifdef DEBUG
   		printf(" HMM j %d sum %f\n",j+1,tmp);
#endif

                i=j;
        }
   }

#ifdef DEBUG
   printf("tmp %f threshold %f\n",tmp,threshold);
#endif 

   if(tmp < threshold){
 		rand=generaR(0.0,5.0);
		i=(int) rand; //it generates a random output 
#ifdef DEBUG
		printf("i %d\n",i);
#endif
   }			
   else i--;
   //printf(" stochastic output symbol %d %s tmp %f\n",i,outputs[i],tmp);

   return i;

}

/* Function that calculates the HMM output from the C matrix */
int calculate_hmm_output_obs(struct hmm_database hmm,int state, int obs)
{
 int i,j;
 float rand;
 float tmp;
 int index;
 char outputs[10][15] = { "STOP","FORWARD","BACKWARD","LEFT","RIGHT"};


 index= (state-1) * hmm.num_obs_symbols + (obs + 1);

 // it generates a Uniform random variable between 0 and 1
 rand=generaR(0.0,1.0);
 //printf("output random %f index %d\n",rand,index);


 tmp=hmm.cis[index][1];
 //printf(" HMM 1 sum %f\n",tmp);

 for(j=1;j<=hmm.num_out_symbols;j++){
        if(rand <= tmp){
                i=j;
                j=hmm.num_out_symbols+1;
        } else{
                tmp= tmp + hmm.cis[index][j+1];
                //printf(" HMM j %d sum %f\n",j+1,tmp);
                i=j;
        }
   }

   i--;
   //printf(" stochastic output symbol %d %s tmp %f\n",i,outputs[i],tmp);

   return i;

}


/* Function that calculates the HMM next state from the A matrix */
int calculate_hmm_next_state(struct hmm_database hmm,int state)
{
 int i,j;
 float rand;
 float tmp;


 // it generates a Uniform random variable between 0 and 1
 rand=generaR(0.0,1.0);
 //printf(" Next state random %f\n",rand);


 tmp=hmm.ais[state][1];
 //printf(" HMM next state 1 sum %f\n",tmp);

 for(j=1;j<=hmm.num_states;j++){
        if(rand <= tmp){
                i=j;
                j=hmm.num_states+1;
        } else{
                tmp= tmp + hmm.ais[state][j+1];
                //printf(" HMM next state j %d sum %f\n",j+1,tmp);
                i=j;
        }
   }

   //printf(" HMM next state %d tmp %f\n",i,tmp);

   return i;

}



/* Function that reads the ais,bis and pis of a hmm */
void read_hmms(struct hmm_database *hmm, int flg_read,char *path, int flg_inputs)
{
   FILE *fp2;
   float average,tmp;
   int j,k;
   char input_file[300];
   int n_states,n_obs_symbols,n_out_symbols;
   int n_bits_inputs;
   int limits;


   /* It opens the hmm file */
   sprintf(input_file,"%s%s",path,hmm->objects_file);
   //printf("Reading HMM FILE: %s\n",input_file);
   fp2=fopen(input_file,"r");
   if(fp2==0){
        printf("\n File %s does not exists",input_file);
        exit(0);
   }

   /* It reads the number of states, observation symbols and output symbols */
   fscanf(fp2,"%d",&n_states);
   hmm->num_states=n_states;
   fscanf(fp2,"%d",&n_obs_symbols);
   hmm->num_obs_symbols=n_obs_symbols;
   fscanf(fp2,"%d",&n_out_symbols);
   hmm->num_out_symbols=n_out_symbols;


   if(flg_read==0) {
		fclose(fp2);
#ifdef DEBUG
   		printf("Num.states %d num.obs_symbols %d num_out_symbols %d\n",n_states,n_obs_symbols,n_out_symbols); 
#endif
		return;
   }

   // It reads the initial probabilities 
   average=0;
   for(j=1;j<=n_states;j++){
        fscanf(fp2,"%f",&tmp);
        hmm->pis[j]=tmp;
        //printf(" pi[%d] %f\n",j,hmm->pis[j]);
        average+=hmm->pis[j];
   }

   //printf("\n sum pis %f\n",average);

   // It reads the transition probabilities 
   for(j=1;j<=n_states;j++){
        average=0;
        for(k=1;k<=n_states;k++){
                fscanf(fp2,"%f",&tmp);
                hmm->ais[j][k]=tmp;
          //      printf("\n a[%d][%d] %f",j,k,hmm->ais[j][k]);
                average+=hmm->ais[j][k];
        }
        //printf("\n %d.- sum= ais %f\n",j,average);
   }

   for(j=1;j<=n_states;j++){
        average=0;
        for(k=1;k<=n_obs_symbols;k++){
                     fscanf(fp2,"%f",&tmp);
                     hmm->bis[j][k]=tmp;
                     //printf(" b[%d][%d] %f\n",j,k,hmm->bis[j][k]);
                     average+=hmm->bis[j][k];
        }
        //printf("sum bis[%d] %f\n",j,average);
   }

   if(flg_inputs==1) limits=n_states*n_obs_symbols;
   else limits=n_states;

   for(j=1;j<=limits;j++){
        average=0;
        for(k=1;k<=n_out_symbols;k++){
                     fscanf(fp2,"%f",&tmp);
                     hmm->cis[j][k]=tmp;
                     //printf(" c[%d][%d] %f\n",j,k,hmm->cis[j][k]);
                     average+=hmm->cis[j][k];
        }
        //printf("sum cis[%d] %f\n",j,average);
   }

   fclose(fp2);


}





void q_free(struct vector_quantizer *vq) {
        free(vq->vectors);
        free(vq->centroids_vision);
        free(vq->centroids);
}

void hmm_free(struct hmm_database *hmm) {
        int i,j;

       	free(hmm->seq_states);
        free(hmm->ais);
        free(hmm->bis);
        free(hmm->cis);
        free(hmm->pis);
}






/* It calculates the probability of an hmm given an observation vector */
double get_hmm_prob(int *obs,float **a,float **b,float *pi,int num_obs,int num_states)
{
   int dummy;
   int t,i,j,k;
   double tran,temp;
   double menor,**alfa,ct,p,ctexp;



   if((alfa=(double **)alloc_matrix_double(num_obs,num_states+1)) == NULL){
         printf("\n Memory allocation error ");
         exit(0);
   }
   t=1;
   ct=0.;

   for(i=1;i<=num_states;i++){
                alfa[t][i]=pi[i]*b[i][obs[t]];
   }


   tran=0.;
   for(i=1;i<=num_states;i++)tran=tran+alfa[t][i];

   for(i=1;i<=num_states;i++)alfa[t][i]=alfa[t][i]/tran;

   tran=1/tran;
   ct=log(tran);
   t=t+1;

   //printf(" num_obs %d\n",num_obs);

   for(dummy=2;dummy<num_obs;dummy++){

                //printf("obs[%d] %d\n",t,obs[t]);

                for(j=1;j<=num_states;j++){

                        tran=0.;
                        for(i=1;i<=num_states;i++){
                                tran=alfa[t-1][i]*a[i][j]+tran;
                        }

                        alfa[t][j]=tran*b[j][obs[t]];
                }

                tran=0.;

                for(j=1;j<=num_states;j++)tran=tran+alfa[t][j];

                for(j=1;j<=num_states;j++){
                        alfa[t][j]=alfa[t][j]/tran;
                }

                tran=1/tran;
                ct=ct+log(tran);
                t++;
   }

   p= exp(-ct);
   //printf("Probability log(P) %f P %e ",-ct,p);
   p= -ct;
   // The result is divided by 100 in order that the result fits the variable size
   p=exp(p/100.);
   //printf("P/100 %e\n",p);

   free_matrix_double(alfa,num_obs);

   return(-ct);
   //return(-ct/100);

}



/* It calculates the best sequence of states using the Viterbi algorithm
and the probability of a hmm given a symbol vector */
double viterbi_best_sequence(int *ob, float **a, float **b, float *pi, int num_obs, int num_states, int *qs, int flg_old)
{
   int t,i,j;
   double bigger=0;
   double **delta,*trans;
   int *q,indice=0,tmay=0,**psi;
   static double delta_old[NUM_STATES];
   float add;
   float sum;



 /* It allocates the matrices and vectors used */
    if((psi =(int **)alloc_matrix_int(num_obs+2,num_states+1)) == 0){
         printf("\n Memory allocation error ");
         exit(0);
    }

    if((delta=(double **)alloc_matrix_double(num_obs+2,num_states+1)) == 0){
         printf("\n Memory allocation error ");
         exit(0);
    }


    if((trans=(double *)alloc_vector_double(num_states+1)) == 0){
         printf("\n Memory allocation error ");
         exit(0);
    }

    if((q =(int *)alloc_vector_int(num_obs+2)) == 0){
         printf("\n Memory allocation error ");
         exit(0);
    }

  for(j=0;j<=num_states;j++){
        trans[j]=0;
  }
  for(i=0;i<=num_obs;i++){
        for(j=0;j<=num_states;j++){
                psi[i][j]=0;
                delta[i][j]=0;
        }
        q[i]=0;
  }




  /* Viterbi Algorithm */
   tmay=num_obs;

   //printf(" ob[%d] %d\n",1,ob[1]);

   for(j=1;j<=num_states;j++){
		//it checks if its the first time the Viterbi is performed
		if(flg_old==1){
                	delta[1][j]=(double)pi[j]*b[j][ob[1]];
                	psi[1][j]= 0;
		}
                else{ // Next block
			bigger=0.;
                        for(i=1;i<=num_states;i++){
                          trans[i]=delta_old[i]*(double)a[i][j];
                          printf("\n next delta[%d][%d] %e",1,i,delta_old[i]);
                          printf("\n a[%d][%d] %f",i,j,a[i][j]);
                          printf("\n b[%d][%d] %f",j,ob[t],b[j][ob[t]]);
                          printf("\n trans[%d] %e",i,trans[i]);
                          if(trans[i]>bigger){
                                        bigger=trans[i];
                                        indice=i;
                          }

                        }

                        delta[1][j]=bigger*b[j][ob[1]];
                        psi[1][j]=indice;

             	}


		//printf("probability delta[%d][%d] %f\n",1,j,delta[1][j]);
                //printf("b[%d][%d] %f\n",j,ob[1],b[j][ob[1]]);
   }

   bigger=0.;
   for(j=1;j<=num_states;j++){
	if(delta[1][j] > bigger){
		 indice=j;
		 bigger = delta[1][j];
	}
   }

   //printf("1. state %d \n",indice);

   for(t=2;t<=tmay;t++){

		// it scaled the previous deltas
                bigger=0.;
                sum=0;
                for(j=1;j<=num_states;j++){
                        //printf("delta{%d][%d] %f\n",t-1,j,delta[t-1][j]);
                        sum= sum + delta[t-1][j];

                        if(delta[t-1][j]>bigger){
                                bigger=delta[t-1][j];
                        }
                }

                add=0;
                // it saves the scaled deltas for the next iteration
		bigger=0;
                for(j=1;j<=num_states;j++){
                        delta[t-1][j]=delta[t-1][j]/sum;
                        add = add + delta[t-1][j];
                        //printf("probability state[%d][%d] %f\n",t-1,j,delta[t-1][j]);
			if(delta[t-1][j] > bigger){
				bigger = delta[t-1][j];
				indice = j;
			}
                }
                //printf("add %f\n",add);
       		//printf("%d. next state %d \n",t-1,indice);

                //printf("ob[%d] %d\n",t,ob[t]);

                for(j=1;j<=num_states;j++){
                        bigger=0.;
                        for(i=1;i<=num_states;i++){
                          trans[i]=delta[t-1][i]*
                                        (double)a[i][j];
                          //printf("\n delta[%d][%d] %e",t-1,i,delta[t-1][i]);
                          //printf("\n a[%d][%d] %f",i,j,a[i][j]);
                          //printf("\n trans[%d] %e",i,trans[i]);
                          if(trans[i]>bigger){
                                        bigger=trans[i];
                                        indice=i;
                          }

                        }

                        //printf("b[%d][%d] %f\n",j,ob[t],b[j][ob[t]]);
                        delta[t][j]=bigger*b[j][ob[t]];
                        //printf("bigger %e\n",bigger);
                        //printf("probability delta[%d][%d] %e\n",t,j,delta[t][j]);
                        psi[t][j]=indice;
               		//printf("%d. j %d next state %d \n",t,j,indice);

                }

   }


   bigger=0;
   for(j=1;j<=num_states;j++){
	if(delta[tmay][j] > bigger){
		bigger=delta[tmay][j];
		indice=j;
	}
   }


   q[tmay]=indice;
   qs[tmay]=indice;

   for(t=tmay-1;t>0;t--){
                q[t]=psi[t+1][q[t+1]];
                qs[t]=q[t];
   }

 /* 
   for(t=1;t<=tmay;t++){
                printf("%d.- symbol %d",t,ob[t]);
                printf(" state %d \n",q[t]);
   }
 */


   bigger=0.;
   for(j=1;j<=num_states;j++){

	//printf("delta[%d][%d] %e\n",tmay,j,delta[tmay][j]);
	if(delta[tmay][j]>bigger)
		bigger=delta[tmay][j];

   }

   for(j=1;j<=num_states;j++){

        // it saves the scaled deltas for the next iteration
        delta_old[j]=delta[tmay][j]/bigger;
        //printf("delta[%d][%d] %e\n",tmay,j,delta[tmay][j]);
        //printf("delta_old[%d][%d] %e\n",tmay,j,delta_old[j]);

   }



   //printf("probability %e\n",bigger);

   free_matrix_double(delta,num_obs+2);
   free_matrix_int(psi,num_obs+2);
   free_vector_double(trans);
   free_vector_int(q);

   return(bigger);

}


int get_next_state_hmm_viterbi(int o, int flg_old, struct hmm_database hmm, int state)
{
  int t,i,j,ob;
  float bigger=0;
  float sum=0;
  float add=0;
  int indice=0,tmay=0;
  static float delta_old[NUM_STATES];
  int psi[NUM_STATES];
  float delta[NUM_STATES];
  float trans;
  int next_state;
  int next_state_region;
  int k;



  ob=o+1;
#ifdef DEBUG
  printf(" ob %d previous state %d\n",ob,state);
#endif
  k=state;

  //it checks if its the first time 
  if(flg_old==1){
        bigger=0.;
        for(j=1;j<=hmm.num_states;j++){
                delta[j]=(double)hmm.pis[j]*hmm.bis[j][ob];
                if(delta[j]>bigger){
                          bigger=delta[j];
               }
               //printf("pis[%d] %f bis[%d][%d] %f\n",j,hmm.pis[j],j,ob,hmm.bis[j][ob]);
               //printf("delta[%d] %e\n",j,delta[j]);
                //next_state=state;
        }
  }
  else{
        // it gets the state according the HMM A matrix

        for(j=1;j<=hmm.num_states;j++){
            bigger=0.;
            for(i=1;i<=hmm.num_states;i++){
               //trans=delta_old[i]*(double)hmm.ais[i][j]*hmm.bis[j][ob];
               trans=delta_old[i]*(double)hmm.ais[i][j];
               //printf("delta_old[%d] %f trans %f\n",i,delta_old[i],trans);
               //printf("ais[%d,%d] %f\n",i,j,hmm.ais[i][j]);
               if(trans > bigger){
                          bigger=trans;
               }
	    }
	    //printf("\n");
            //delta[j]=bigger;
            delta[j]=bigger*hmm.bis[j][ob];
            delta[j]=bigger*hmm.bis[j][ob];
            //printf("bis[%d][%d] %f\n",j,ob,hmm.bis[j][ob]);
            //delta[j]=bigger*hmm.bis[k][ob];
            //printf("bis[%d][%d] %f\n",k,ob,hmm.bis[k][ob]);
            //printf("delta[%d] %f bigger %f\n\n",j,delta[j],bigger);

         }
  }

  bigger=0.;
  sum=0;
  for(j=1;j<=hmm.num_states;j++){
        //printf("delta[%d] %f\n",j,delta[j]);
	sum= sum + delta[j];

        if(delta[j]>bigger){
                bigger=delta[j];
                next_state=j;
	}
  }


  add=0;
  // it saves the scaled deltas for the next iteration
  for(j=1;j<=hmm.num_states;j++){
        //delta_old[j]=delta[j]/bigger;
        delta_old[j]=delta[j]/sum;
	add = add + delta_old[j]; 
#ifdef DEBUG
        printf("probability next state[%d] %f\n",j,delta_old[j]);
#endif
   }
   //printf("add %f\n",add);


   next_state_region=calculate_region(delta_old,hmm.num_states);

#ifdef DEBUG
   printf(" Viterbi best state %d\n",next_state);
   printf(" Viterbi prob. region state %d\n",next_state_region);
#endif

   //return(next_state_region);
   return(next_state);

}


// It calculates the next state of a HMM given the present state and a symbol 
int get_next_state_hmm(int o, int flg_old, struct hmm_database hmm, int state)
{
  int t,i,j,ob;
  double bigger=0;
  int indice=0,tmay=0;
  static double delta_old[NUM_STATES];
  int psi[NUM_STATES];
  double delta[NUM_STATES];
  double trans[NUM_STATES];
  int next_state;

  ob=o+1;
  printf(" ob %d\n",ob);

  //it checks if its the first time 
  if(flg_old==1){
        bigger=0.;
   	for(j=1;j<=hmm.num_states;j++){
         	delta[j]=(double)hmm.pis[j]*hmm.bis[j][ob];
		if(delta[j]>bigger){
                          bigger=delta[j];
               }

	 	//printf("pis[%d] %f bis[%d][%d] %f\n",j,hmm.pis[j],j,ob,hmm.bis[j][ob]);
	 	//printf("delta[%d] %e\n",j,delta[j]);
		next_state=state;
   	}
  }
  else{
  	// it gets the state according the HMM A matrix
  	i=calculate_hmm_next_state(hmm,state);
	printf(" Calculated state %d\n",i);

        bigger=0.;
        for(j=1;j<=hmm.num_states;j++){
               delta[j]=delta_old[j]*(double)hmm.ais[i][j]*hmm.bis[j][ob];
	       //printf("delta_old[%d] %e\n",j,delta_old[j]);
	       //printf("ais[%d,%d] %f bis[%d][%d] %f\n",i,j,hmm.ais[i][j],j,ob,hmm.bis[j][ob]);
	       //printf("delta[%d] %e\n",j,delta[j]);
               if(delta[j]>bigger){
                          bigger=delta[j];
   			  next_state=j;
               }
   	       //printf(" next state %d %d\n",j,next_state);
         }
  }

  // it saves the scaled deltas for the next iteration
  for(j=1;j<=hmm.num_states;j++){
        delta_old[j]=delta[j]/bigger;
   }

   //printf(" next state %d\n",next_state);

   return(next_state);

}





/* It calculates the best sequence of states using the Viterbi algorithm
and the probability of a hmm given a symbol vector */
double viterbi(int *observations, float **a, float **b, float *pi, int num_obs, int num_states, int *seq_states)
{
   int i,j,k;
   int *observations_frame,*seq_states_frames;
   int num_frames;
   double partial_prob,total_probability=0;
   int flg_old=1;


 /* it was necessary to separated the observation vector in
    small vectors to better use the Viterbi algorithm, due to
    errors introduced by small acumulated probabilities values */

 num_frames = num_obs/NUM_MAX_OBSERVATIONS;


 // it allocates the observations vectors //
 if((observations_frame = (int *) alloc_vector_int((num_obs+1)) ) == 0){
         printf("\n Memory allocation error: observations ");
         exit(0);
 }

 // it allocates the states vectors
 if((seq_states_frames = (int *) alloc_vector_int((num_obs+1))) == 0){
                printf("\n Memory allocation error: observations ");
                exit(0);
 }


 for(k=0; k< num_frames; k++){
        for(j=k*NUM_MAX_OBSERVATIONS+1,i=1;i<=NUM_MAX_OBSERVATIONS;i++,j++)
                observations_frame[i]= observations[j];

        partial_prob=
		viterbi_best_sequence(observations_frame,a,b,pi,NUM_MAX_OBSERVATIONS,num_states,seq_states_frames,flg_old);
		flg_old=0;

	total_probability=total_probability+log(partial_prob);
 	printf("block %d partial_prob %e log total_probability %e\n",k,partial_prob,total_probability);

        for(j=k*NUM_MAX_OBSERVATIONS+1,i=1;i<=NUM_MAX_OBSERVATIONS;i++,j++)
                seq_states[j]= seq_states_frames[i];

 }
 for(j=k*NUM_MAX_OBSERVATIONS+1,i=1;i<=num_obs-k*NUM_MAX_OBSERVATIONS;i++,j++)
                observations_frame[i]= observations[j];

 partial_prob=
	viterbi_best_sequence(observations_frame,a,b,pi,num_obs-k*NUM_MAX_OBSERVATIONS, num_states,seq_states_frames,flg_old);

 total_probability=total_probability+log(partial_prob);
 //printf(" partial_prob %e log total_probability %e\n",partial_prob,total_probability);

 for(j=k*NUM_MAX_OBSERVATIONS+1,i=1;i<=num_obs-k*NUM_MAX_OBSERVATIONS;i++,j++)
                seq_states[j]= seq_states_frames[i];


/*
 for(j=1;j<=num_obs;j++){
                printf("%d.- symbol %d",j,observations[j]);
                printf(" state %d \n",seq_states[j]);
 }
*/ 

 return(total_probability);


}


// It updates the hmm probabilities using the Viterbi algorithm
int get_statistics_HMM(int num_train,int *num_vectors,int ** obs, hmm_database *hmm, int **q,  int **actions)
{
 int i,j,k,l,t;
 float tmp,lim,average;
 int num_obs[NUM_STATES+1][NUM_MAX_CENTROIDS+1];
 int num_nds,ret_nodes[256];
 int num_states_obs[NUM_STATES+1];
 int state_obs[NUM_STATES+1][NUM_STATES+1];
 int t1;
 int inc,init,end;
 float **a,*pis,**b;
 double vit_prob[1024];
 int *vect,**vectk;
 int *vect_pi;
 int **vect_out;
 int state[NUM_STATES+1][NUM_STATES+1];
 double p;
 int num_states;
 int num_symbols;
 int num_outputs;
 int lim_pis;
 int sum;
 int index;
 int previous_j=1;
 float add,threshold=0.1;
 float value;


 num_states=hmm->num_states;
 num_symbols=hmm->num_obs_symbols;
 num_outputs=hmm->num_out_symbols;


 // it allocates memory
 if((vect_pi =(int *)alloc_vector_int(num_states+1)) == 0){
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

 if((vect_out=(int **)alloc_matrix_int(num_states*num_symbols+num_symbols+1,num_outputs+1))==0){
         printf("\n Memory allocation error ");
         exit(1);
 }

 // it initializes some variables
 for(i=1;i<=num_states;i++){
        vect[i]=0;
        vect_pi[i]=0;
        for(j=1;j<=num_states;j++){
                state[i][j]=0;
        }
        for(j=1;j<=num_symbols;j++){
                        vectk[i][j]=0;
        }

 }

 for(i=1;i<=num_states*num_symbols;i++)
        for(j=0;j<num_outputs;j++)
		vect_out[i][j]=0;


 for(i=1;i<=num_train;i++){

        vect_pi[q[i][1]]++;
	//printf("%d state %d vect_pi %d\n",i,q[i][1],vect_pi[q[i][1]]); 
        for(t=1;t<num_vectors[i];t++){
                state[q[i][t]][q[i][t+1]]++;
                vect[q[i][t]]++;
                vectk[q[i][t]][obs[i][t]]++;

		index= (q[i][t]-1) * num_symbols + obs[i][t] ;
                vect_out[index][actions[i][t]]++;
                //printf("state[%d][%d] %d obs[%d][%d] %d\n",q[i][t],q[i][t+1],state[q[i][t]][q[i][t+1]],i,t,obs[i][t]);
		//printf("index %d action[%d][%d] %d\n",index,i,t,actions[i][t]);
        }
 }


 /* It updates the pis, ais and bis */
 for(i=1;i<=num_states;i++){

                hmm->pis[i] = (float)(vect_pi[i]/(float)num_train);
                //printf("p[%d] %f\n",i,hmm->pis[i]);

                for(j=1;j<=num_states;j++){
                        if(vect[i] != 0)
                        hmm->ais[i][j]=(float)(state[i][j]/(float)vect[i]);
                        else hmm->ais[i][j]=0.;
                        printf("a[%d][%d] %f\n",i,j,hmm->ais[i][j]);
                }
                for(j=1;j<=num_symbols;j++){
                        if(vect[i] != 0)
                                hmm->bis[i][j]=(float)(vectk[i][j]/(float)vect[i]);
                        else    hmm->bis[i][j]=0.;
                        printf("\n b[%d][%d] %f",i,j,hmm->bis[i][j]);
                }
 }

  
 /* It updates the cis */
 for(i=1;i<=num_states*num_symbols;i++){
	sum=0;
	add=0;
        for(j=0;j<num_outputs;j++)sum= sum + vect_out[i][j];
	for(j=0;j<num_outputs;j++){
		//printf("vect_out[%d][%d] %d\n",i,j,vect_out[i][j]);
                if(vect_out[i][j] != 0){
			hmm->cis[i][j+1]=(float)(vect_out[i][j]/(float)sum);
			previous_j=j;
		}
		else    hmm->cis[i][j+1]= 0.01;
                //printf("c[%d][%d] %f\n",i,j+1,hmm->cis[i][j+1]);
		add = add + hmm->cis[i][j+1];
	}
	if(add < threshold){
                printf("Retrain more for Matriz C %d \n",i);

		for(j=0;j<num_outputs;j++){
			hmm->cis[i][j+1]= 0.1 / (num_outputs - 1);
		}
		hmm->cis[i][previous_j]=0.90;
                //printf("c[%d][%d] %f\n",i,previous_j,hmm->cis[i][previous_j]);
	}
 }


 lim_pis=0;

 // It checks for probabilities values less than NULL_PROB and fixed it
 for(k=1;k<=num_states;k++){

        if(hmm->pis[k] < NULL_PROB){
                  hmm->pis[k]=NULL_PROB;
		  lim_pis++;
	}

        lim=0;
        for(l=1;l<=num_states;l++){
              if(hmm->ais[k][l]<NULL_PROB){
                  hmm->ais[k][l]=NULL_PROB;
                  lim=lim+1;
                  //printf("%d %d lim %f\n",k,l,lim);
                  printf("fixing a[%d][%d] %f\n",k,l,hmm->ais[k][l]);
              }
        }
        /*it fixes the probabilities in order that the total probability is 1 */
        if(lim != 0){
          if(lim == num_states){
                 printf("Retrain more for matriz A %d\n",k);
		 //l= (int) generaR(0.0,(float) num_states);
		 //hmm->ais[k][l]= 1.0 - (num_states-1)*NULL_PROB;

		value = 1.0 / (float) num_states;
		for(l=1;l<=num_states;l++){
			hmm->ais[k][l]= value;
		}

                //exit(0);
          }


          for(l=1;l<= num_states;l++)
            if((hmm->ais[k][l]!=NULL_PROB)&&(hmm->ais[k][l]>(lim*NULL_PROB)/(num_states-lim))){
                //hmm->ais[k][l]=hmm->ais[k][l]-(lim*NULL_PROB)/(float)(num_states-lim);

                  printf("fixing_2 a[%d][%d] %f\n",k,l,hmm->ais[k][l]);
	      }
        }

   }


 //it fixes the pi probabilities
 for(l=1;l<= num_states;l++){
      if((hmm->pis[l]!=NULL_PROB)&&(hmm->pis[l]>(lim_pis*NULL_PROB)/(num_states-lim_pis)))
                hmm->pis[l]=hmm->pis[l]-(lim_pis*NULL_PROB)/(float)(num_states-lim_pis);

      //printf("p[%d] %f\n",l,hmm->pis[l]);
 }


 for(k=1;k<=num_states;k++){
        lim=0;
        for(l=1;l<=num_symbols;l++){
               if(hmm->bis[k][l]<=NULL_PROB){
                  hmm->bis[k][l]=(float) NULL_PROB;
                  lim++;
                }
        }

        //it fixes the probabilities in order that the total probability is 1
        if(lim != 0){
          for(l=1;l<= num_symbols;l++)
            if((hmm->bis[k][l] > NULL_PROB)
              &(hmm->bis[k][l] > ((lim*NULL_PROB)/(float)(num_symbols-lim)) )){

               hmm->bis[k][l]=hmm->bis[k][l]-(lim*(float)NULL_PROB)/(float)(num_symbols-lim);

            }

        }
 }



 for(i=1;i<=num_states;i++){
        for(j=1;j<=num_states;j++){
                        printf("HMM a[%d][%d] %f\n",i,j,hmm->ais[i][j]);
        }
        for(j=1;j<=num_symbols;j++){
                        printf("HMM b[%d][%d] %f\n",i,j,hmm->bis[i][j]);
        }
 }





 free_vector_int(vect_pi);
 free_vector_int(vect);
 free_matrix_int(vectk,num_states+1);
 free_matrix_int(vect_out,(num_states*num_symbols)+num_symbols+1);



}



void fprint_hmm(char *path, hmm_database *hmm){

  int i,j;
  int num_states,num_symbols,num_outputs;
  float sum_prb;
  FILE *fp;
  int m,n;
  char file_name[300];


  strcpy(file_name,path);
  strcat(file_name,hmm->objects_file);
  strcat(file_name,"_new.prb");
  //printf("New probability file %s\n",file_name);



  if ((fp=fopen(file_name,"w"))==NULL){
                fprintf(stderr,"Error opening file %s\n",file_name);
                exit(1);
  }


 num_states=hmm->num_states;
 num_symbols=hmm->num_obs_symbols;
 num_outputs=hmm->num_out_symbols;



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
        fprintf(fp,"%f ",hmm->pis[n]);
        //printf("pi[%d] %f\n",n,hmm->pis[n]);
        sum_prb=sum_prb+hmm->pis[n];
        m++;
   }
   fprintf(fp,"\n\n");
   //printf(" sum. prb Pi %f\n",sum_prb);

   // it saves the A matrix
  for(j=1; j <= num_states; j++){
        sum_prb=0.0;
        for(n=1; n <= num_states; n++){
                fprintf(fp,"%f ",hmm->ais[j][n]);
                printf("a[%d][%d] %f\n",j,n,hmm->ais[j][n]);
                sum_prb=sum_prb+hmm->ais[j][n];
                m++;
        }
        fprintf(fp,"\n");
        //printf(" sum. prb A %d %f\n",j,sum_prb);

  }
  fprintf(fp,"\n");


  // it saves the B matrix
  for(j=1; j <= num_states; j++){
        sum_prb=0.0;
        for(n=1; n <= num_symbols; n++){
                fprintf(fp,"%f ",hmm->bis[j][n]);
                printf("b[%d][%d] %f\n",j,n,hmm->bis[j][n]);
                sum_prb=sum_prb+hmm->bis[j][n];
                m++;
        }
        fprintf(fp,"\n");
        //printf("sum. prb B %d %f\n",j,sum_prb);
  }
  fprintf(fp,"\n");

    // it saves the C matrix
  for(j=1; j <= num_states*num_symbols; j++){
        sum_prb=0.0;
        for(n=1; n <= num_outputs; n++){
                fprintf(fp,"%f ",hmm->cis[j][n]);
                //printf("C[%d][%d] %f\n",j,n,hmm->cis[j][n]);
                sum_prb=sum_prb+hmm->cis[j][n];
                m++;
        }
        fprintf(fp,"\n");
        //printf(" sum. prb C %d %f\n",j,sum_prb);
  }


  fclose(fp);

}


