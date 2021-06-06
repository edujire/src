/********************************************************
 *                                                      *
 *                                                      *
 *      reactive_navigation.h		          	*
 *                                                      *
 *		Jesus Savage				*
 *		FI-UNAM					*
 *		4-10-2019                               *
 *                                                      *
 ********************************************************/

/*
#define NUM_BITS_INPUTS 5
#define NUM_BITS_OUTPUTS 3 // bits to decode 5 outputs: STOP, BACKWARD, FORWARD, TURN_LEFT, TURN_RIGHT
#define NUM_MAX_MEMORY 65535 // 2 >> 16
#define REAL float
*/

    
/*typedef struct _memory_state_machine{
        int state;
        int output;
} memory_state_machine;
*/

struct pr_table {
        float **ais;
        int num_inputs;
        int num_outputs;
	int num_states;
};


float **alloc_matrix(int row,int col)
{
    int i;
    float **m;

    m = (float **) malloc((unsigned) row*sizeof(float *));
    for (i=0; i<row; i++)
        m[i] = (float *) malloc((unsigned) col*sizeof(float));
    return m;
}


/* Function that calculates the output from a probability action matrix */
int calculate_stochastic_action_output(pr_table pr_actions, int state)
{
 int i,j;
 float rand;
 float tmp;
 int index;


 index= state;

 // it generates a Uniform random variable between 0 and 1
 rand=generaR(0.0,1.0);

 tmp=pr_actions.ais[index][1];
 //printf("pr 1 sum %f\n",tmp);

 for(j=1;j<=pr_actions.num_outputs;j++){
        if(rand <= tmp){
                i=j;
                j=pr_actions.num_outputs+1;
        } else{
                tmp= tmp + pr_actions.ais[index][j+1];
                //printf("pr j %d sum %f\n",j+1,tmp);
                i=j;
        }
   }

   i--;
   //printf(" stochastic output symbol %d %s tmp %f\n",i,outputs[i],tmp);

   return i;

}





// it reads the actions' probabilities
int  read_reactive_pr_table(char *file, pr_table *pr){

 int i,j,k;
 FILE *fpr;
 int flag=1;
 int x,num_outputs,num_inputs;
 int state=0,output=0;
 float tmp,average;

#ifdef DEBUG
 printf("File %s\n",file);
#endif

 if((fpr=fopen(file,"r")) == NULL){
        printf("File %s can not be open\n",file);
        exit(0);
 }

 fscanf(fpr,"%d",&num_outputs);
 pr->num_outputs=num_outputs;
 fscanf(fpr,"%d",&num_inputs);
 pr->num_inputs=num_inputs;

#ifdef DEBUG
 printf(" num_outputs %d num_inputs %d \n",pr->num_outputs,pr->num_inputs);
#endif

 
 // It allocates the matriz used for the actions' generations
 if ((pr->ais = (float **)alloc_matrix(num_inputs + 1, num_outputs + 1 )) == 0) {
                        printf("\n Memory allocation error ");
                        exit(1);
 }


 // It reads the action probabilities 
 for(j=1;j<=num_inputs;j++){
        average=0;
        for(k=1;k<=num_outputs;k++){
                fscanf(fpr,"%f",&tmp);
                pr->ais[j][k]=tmp;
#ifdef DEBUG
                printf("\n a[%d][%d] %f",j,k,pr->ais[j][k]);
#endif
                average+=pr->ais[j][k];
        }
#ifdef DEBUG
        printf("\n %d.- sum= ais %f\n",j,average);
#endif
 }

 return num_outputs;

}



int  read_reactive_table(char *file,int *num_bits_out,int *num_in,int *memory_output){

 int i,j;
 FILE *fpr;
 int flag=1;
 int x,num_bits_state,num_bits_output,num_inputs;
 int state=0,output=0;

#ifdef DEBUG
 printf("File %s\n",file);
#endif

 if((fpr=fopen(file,"r")) == NULL){
        printf("File %s can not be open\n",file);
        exit(0);
 }

 fscanf(fpr,"%d",&num_bits_output);
 fscanf(fpr,"%d",&num_inputs);
#ifdef DEBUG
 printf(" Num. bits output %d num_inputs %d \n",
                                num_bits_output,num_inputs);
#endif

 j=1;
 while(flag){

	output=0;
        for(i=1;i<=num_bits_output;i++){

                if(fscanf(fpr,"%d",&x) == EOF)
                        flag=0;
                else{
                        output= (x << (num_bits_output-i)) + output;
                }
        }


       	memory_output[j]=output;
#ifdef DEBUG
       	printf("memory[%d] output %d\n",j,memory_output[j]);
#endif
        j++;

        if(j==NUM_MAX_MEMORY){
                printf(" Increase constant NUM_MAX_MEMORY in file reactive_navigation.h\n");
                exit(0);
        }


 }

 j--;
#ifdef DEBUG
 printf("Num. memory locations %d\n",j-1);
 printf("Num. total memory %d\n",2<<(num_inputs-1));
#endif

 fclose(fpr);

 *num_in=num_inputs;
 *num_bits_out=num_bits_output;

 return(2<<(num_inputs-1));

}




