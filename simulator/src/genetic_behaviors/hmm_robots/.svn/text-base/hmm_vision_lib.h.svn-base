// hmm_vision_lib.h
// Basic HMM libraries
//
//	Jesus Savage
//	FI-UNAM
//	4-24-2009
//

int read_objects_name(char *objects_file,char **objects_name, char *path)
{
 char input_file[300],name[300];
 FILE *fp2;
 int i;


 /* It opens the file that contains the names of the objects to be recognized */
 sprintf(input_file,"%s%s.txt",path,objects_file);

 printf("Objects FILE: %s\n",input_file);

 fp2=fopen(input_file,"r");
 if(fp2==0){
             printf("\n Can not open %s ",input_file);
             return(0);
 }

 i=1;

 /* it reads the names */
 while(fscanf(fp2,"%s",name) != EOF){
         strcpy(objects_name[i],name);
         i++;
}

fclose(fp2);

return(i-1);

}


/* Function that reads the ais,bis and pis of a hmm */
void read_hmm_ais_bis_pis(char *object,float **a,float *pi,float **b,int num_states,int num_symbols,char *path)
{
   FILE *fp2;
   float average,tmp;
   int j,k;
   char input_file[300];

   /* It opens the hmm file */
   sprintf(input_file,"%s%s_%d_%d.prb",path,object,num_states,num_symbols);
   //printf("Reading HMM FILE: %s\n",input_file);
   fp2=fopen(input_file,"r");
   if(fp2==0){
        printf("\n File %s does not exists",input_file);
        exit(0);
   }

   /* It reads the transition  and initial probabilities */
   average=0;
   for(j=1;j<=num_states;j++){
        fscanf(fp2,"%f",&tmp);
        pi[j]=tmp;
        //printf("reading pi[%d] %f\n",j,pi[j]);
        average+=pi[j];
   }
   //printf("\n sum pis %f\n",average);

   for(j=1;j<=num_states;j++){
        average=0;
        for(k=1;k<=num_states;k++){
                fscanf(fp2,"%f",&tmp);
                a[j][k]=tmp;
                //printf("\n a[%d][%d] %f",j,k,a[j][k]);
                average+=a[j][k];
        }
        //printf("\n %d.- sum= ais %f",j,average);
   }

   for(j=1;j<=num_states;j++){
        average=0;
        for(k=1;k<=num_symbols;k++){
                     fscanf(fp2,"%f",&tmp);
                     b[j][k]=tmp;
                     //printf("b[%d][%d] %f\n",j,k,b[j][k]);
                     average+=b[j][k];
        }
        //printf("sum bis[%d] %f\n",j,average);
   }

   fclose(fp2);
}



/* It reads the hmm of each of the objects */
void read_hmms(char **rec_objects,int num_objects,float ***ais,float ***bis,float **pis, int num_states, int num_symbols, char *path)
{
 int k;



 for(k=1;k<=num_objects;k++){
        //printf("rec_objects[%d] %s\n",k,rec_objects[k]);

        /* It reads the HMM model of each object */
        read_hmm_ais_bis_pis(rec_objects[k],ais[k],pis[k],bis[k],num_states,num_symbols,path);
 }

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
                else{
			bigger=0.;
                        for(i=1;i<=num_states;i++){
                          trans[i]=delta_old[i]*(double)a[i][j]*b[j][ob[1]];
                          //printf("\n next delta[%d][%d] %e",1,i,delta_old[i]);
                          //printf("\n a[%d][%d] %f",i,j,a[i][j]);
                          //printf("\n b[%d][%d] %f",j,ob[t],b[j][ob[t]]);
                          //printf("\n trans[%d] %f",i,trans[i]);
                          if(trans[i]>bigger){
                                        bigger=trans[i];
                                        indice=i;
                          }

                        }

                        delta[1][j]=bigger;
                        psi[1][j]=indice;

             	}


		/*printf("\n ini delta[%d][%d] %f",1,i,delta[1][i]);
                printf("\n b[%d][%d] %f",i,ob[1],b[i][ob[1]]);*/
   }

   for(t=2;t<=tmay;t++){

                //printf(" ob[%d] %d\n",t,ob[t]);

                for(j=1;j<=num_states;j++){
                        bigger=0.;
                        for(i=1;i<=num_states;i++){
                          trans[i]=delta[t-1][i]*
                                        (double)a[i][j]*b[j][ob[t]];
                          //printf("\n delta[%d][%d] %e",t-1,i,delta[t-1][i]);
                          //printf("\n a[%d][%d] %f",i,j,a[i][j]);
                          //printf("\n b[%d][%d] %f",j,ob[t],b[j][ob[t]]);
                          //printf("\n trans[%d] %f",i,trans[i]);*/
                          if(trans[i]>bigger){
                                        bigger=trans[i];
                                        indice=i;
                          }

                        }

                        delta[t][j]=bigger;
                        psi[t][j]=indice;
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
 	//printf("block %d partial_prob %e log total_probability %e\n",k,partial_prob,total_probability);

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



