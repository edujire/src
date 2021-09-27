/********************************************************
 *                                                      *
 *                                                      *
 *      state_machine_engine.h		          	*
 *                                                      *
 *		Jesus Savage				*
 *		FI-UNAM					*
 *		26-6-2016                               *
 *                                                      *
 ********************************************************/

//#define NUM_BITS_INPUTS 10
#define NUM_MAX_INPUTS 8192// 2 >> NUM_BITS_INPUTS+NUM_BITS_STATES
#define NUM_BITS_OUTPUTS 3 // bits to decode 8 outputs: STOP, BACKWARD, FORWARD, TURN_LEFT, TURN_RIGHT, etc
#define NUM_MAX_OUTPUTS 16
#define NUM_BITS_INTENSITY 2
#define NUM_BITS_DEST_ANGLE 3
#define NUM_MAX_MEMORY 65535 // 2 >> 16
#define THRESHOLD 30
//#define DEBUG 10
    
typedef struct _memory_state_machine{
        int state;
        int output;
} memory_state_machine;


/*
// it converts a binary output to a robot's output
Behavior table_output_destination(int dir){

  Behavior output;
  float angle=(float) (PI/(4.0*15.0));
  float advance;
  int mag;
  float mag_angle;


  advance= Mag_Advance*1.00;
  mag=1;
  mag_angle=8.0;


  switch(dir){

        case 0: // Stop
                output.xc=0.0f;
                output.yc=0.0f;
                printf("STOP\n");
                break;

        case 1: // Forward
                output.xc=advance*mag;
                output.yc=0.0f;
                printf("FORWARD\n");
                break;

        case 2: // backward
                output.xc=-advance*mag;
                output.yc=0.0f;
                Num_backward++;
                printf("BACKWARD\n");
                break;

        case 3:// Turn left
                output.xc=0.0f;
                output.yc=angle*mag_angle;
                printf("LEFT\n");
                break;

        case 4: // Turn right
                output.xc=0.0f;
                output.yc=-angle*mag_angle;
                printf("RIGHT\n");
                break;

        default:printf("Output %d not defined used STOP\n", dir);
                output.xc=0;
                output.yc=0;
                break;
  }

  return(output);

}

*/



int  read_state_machine(char *file,int *memory_state,int *num_bits_out,int *num_in,int *memory_output){

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

 fscanf(fpr,"%d",&num_bits_state);
 fscanf(fpr,"%d",&num_bits_output);
 fscanf(fpr,"%d",&num_inputs);
#ifdef DEBUG
 printf(" Num. bits state %d num. bits output %d num_inputs %d \n",
                                num_bits_state,num_bits_output,num_inputs);
#endif

 j=1;
 while(flag){
        for(i=1;i<=num_bits_state;i++){

                if(fscanf(fpr,"%d",&x) == EOF)
                        flag=0;
                else{
                        state= (x << (num_bits_state-i)) + state;
                }
        }

        for(i=1;i<=num_bits_output;i++){

                if(fscanf(fpr,"%d",&x) == EOF)
                        flag=0;
                else{
                        output= (x << (num_bits_output-i)) + output;
                }
        }


	memory_state[j]=state;
       	memory_output[j]=output;
#ifdef DEBUG
       	printf("memory[%d] state %d output %d\n",j,memory_state[j],memory_output[j]);
#endif

        state=0;
        output=0;


        j++;

        if(j==NUM_MAX_MEMORY){
                printf(" Increase constant NUM_MAX_MEMORY in file ~/robotics/state_machines/state_machine_engine.h\n");
                exit(0);
        }


 }

 j--;
#ifdef DEBUG
 printf("Num. memory locations %d\n",j-1);
 printf("Num. total memory %d\n",2<<(num_bits_state+num_inputs-1));
#endif

 for(i=j;i< (2<<(num_bits_state+num_inputs-1)); i++){

        memory_state[i]=0;
        memory_output[i]=0;
#ifdef DEBUG
        printf("memory[%d] state %d output %d\n",i,memory_state[i],memory_output[i]);
#endif

 }


 fclose(fpr);

 *num_in=num_inputs;
 *num_bits_out=num_bits_output;

 return(2<<(num_bits_state+num_inputs-1));

}



int  read_state_machine_stochastic(char *file,int *memory_state,int *num_in,pr_table *memory_output_pr){

 int i,j,k;
 FILE *fpr;
 int flag=1;
 int x,num_bits_state,num_bits_output,num_inputs;
 int state=0,output=0;
 int num_variables;
 float var;
 float average;


#ifdef DEBUG
 printf("File %s\n",file);
#endif


 if((fpr=fopen(file,"r")) == NULL){
        printf("File %s can not be open\n",file);
        exit(0);
 }

 fscanf(fpr,"%d",&num_bits_state);
 fscanf(fpr,"%d",&num_variables);
 fscanf(fpr,"%d",&num_inputs);

#ifdef DEBUG
 printf(" Num. bits state %d num. variables %d num_inputs %d \n", num_bits_state,num_variables,num_inputs);
#endif



 j=1;
 while(flag){
        for(i=1;i<=num_bits_state;i++){

                if(fscanf(fpr,"%d",&x) == EOF)
                        flag=0;
                else{
                        state= (x << (num_bits_state-i)) + state;
                }
        }

        memory_state[j]=state;

#ifdef DEBUG
        printf("memory[%d] state %d pr_output ",j,memory_state[j]);
#endif

	average=0;
        for(i=1;i<=num_variables;i++){

                if(fscanf(fpr,"%f",&var) == EOF)
                        flag=0;
                else{
                	memory_output_pr->ais[j][i]=var;
#ifdef DEBUG
                	printf("%f ",memory_output_pr->ais[j][i]);
#endif
                	average+=memory_output_pr->ais[j][i];
        	}
	}
#ifdef DEBUG
	printf("\n");
#endif
       	//printf("%d.- sum memory_output_pr %f\n",j,average);
        state=0;
        j++;

        if(j==NUM_MAX_MEMORY){
                printf(" Increase constant NUM_MAX_MEMORY in file ~/robotics/state_machines/state_machine_engine.h\n");
                exit(0);
        }


 }

 j--;
#ifdef DEBUG
 printf("Num. memory locations %d\n",j-1);
 printf("Num. total memory %d\n",2<<(num_bits_state+num_inputs-1));
#endif



 /*for(i=j;i< (2<<(num_bits_state+num_inputs-1)); i++){

        memory_state[i]=0;
	printf("memory[%d] state %d ",i,memory_state[i]);

	for(k=1;k<=num_variables;k++){
		memory_output_pr->ais[i][j]=0.0;
		printf("%f ",memory_output_pr->ais[i][j]);
	}
	printf("\n");
 }
*/

 fclose(fpr);

 *num_in=num_inputs;

 memory_output_pr->num_outputs=num_variables;

 return(2<<(num_bits_state+num_inputs-1));

}


int state_machine_engine(int obs, int dest, int q_intensity, movement *movements, int *next_state,float Mag_Advance,float max_angle,
                     char *path,int num_bits_vq, float intensity, int flg_genetics, int ind, char *file_behavior){

 //AdvanceAngle gen_vector;
 static int *mem_state,*mem_output;
 static int flg_read=1;
 static int num_bits_input=NUM_BITS_INPUTS,num_bits_output=NUM_BITS_OUTPUTS;
 static int actual_ind=-1;
 int size_mem;
 int index;
 int out;
 //Behavior value;
 int input;
 char state_machine_file_name[300];
 int num_bits_intensity = NUM_BITS_INTENSITY;
 int num_bits_dest_angle = NUM_BITS_DEST_ANGLE;
 int result = 0;
 int state = *next_state;
 
 //printf("flag read: %i, ind: %i ind_actul = %i,flg_genetics %i \n",flg_read,ind,actual_ind,flg_genetics);
 if(flg_genetics==1){
	if(ind != actual_ind){//nuevo individuo
		actual_ind=ind;
		sprintf(state_machine_file_name,"%sdata/avoid_fsm_%i.dat",path,actual_ind);
		mem_state= (int *) malloc((unsigned) NUM_MAX_MEMORY*sizeof(int));
        	mem_output= (int *) malloc((unsigned) NUM_MAX_MEMORY*sizeof(int));
		size_mem=read_state_machine(state_machine_file_name,mem_state,&num_bits_output,&num_bits_input,mem_output);
 	}		
}
 else if(flg_read==1){
        mem_state= (int *) malloc((unsigned) NUM_MAX_MEMORY*sizeof(int));
        mem_output= (int *) malloc((unsigned) NUM_MAX_MEMORY*sizeof(int));
    	sprintf(state_machine_file_name,"%sdata/%s",path,file_behavior);
    	//sprintf(state_machine_file_name,"/media/alejandro/archivos/MobileRobotSimulator/catkin_ws/src/simulator/src/genetic_behaviors/state_machines/state_machine_mem.txt");
        size_mem=read_state_machine(state_machine_file_name,mem_state,&num_bits_output,&num_bits_input,mem_output);
    	flg_read=0;
}

#ifdef DEBUG
 printf("quantized intensity %d laser %d angle_dest %d\n",q_intensity,obs,dest);
#endif
 //input = (intensity << 4) + (obs << 2) + dest;
 input = (q_intensity << (num_bits_dest_angle+num_bits_vq)) + (obs << num_bits_dest_angle) + dest;
#ifdef DEBUG
 printf("present state %d input %d\n",state,input);
#endif
 index= (((state ) << num_bits_input) + input+1) & 0xffff ;

 *next_state=mem_state[index];
 out=mem_output[index];
#ifdef DEBUG
 printf("index %d next_state %d out %d\n",index,*next_state,out);
#endif

if (intensity > THRESHOLD){
            *movements=generate_output(STOP,Mag_Advance,max_angle);
            printf("\n **************** Reached light source ******************************\n");
            result = 1;
                }
else 
 // /home/biorobotica/robotics/utilities/utilities.h //
 *movements=generate_output_genetics(out,Mag_Advance,max_angle);
 //*movements=generate_output(out,Mag_Advance,max_angle);
 //printf("Next State: %d\n", *next_state);

 return result;

}

// State Machine engine 
int state_machine_engine_stochastic(int obs, int dest, int q_intensity,movement *movements, int *next_state,float Mag_Advance,float max_angle,
                                    char *path, int num_bits_vq, float intensity){

 //AdvanceAngle gen_vector;
 static int *mem_state,*mem_output;
 static pr_table pr_output;
 static int flg_read=1;
 static int num_bits_input=NUM_BITS_INPUTS,num_bits_output=NUM_BITS_OUTPUTS;
 int size_mem;
 int index;
 int out;
 //Behavior value;
 int input;
 char state_machine_file_name[300];
 int num_bits_intensity = NUM_BITS_INTENSITY;
 int num_bits_dest_angle = NUM_BITS_DEST_ANGLE;
 int num_inputs = NUM_MAX_INPUTS;
 int num_outputs = NUM_MAX_OUTPUTS;
 int result = 0;
 int state = *next_state;


 if(flg_read==1){
        mem_state= (int *) malloc((unsigned) NUM_MAX_MEMORY*sizeof(int));
    
    // It allocates the matriz used for the actions' generations
    if ((pr_output.ais = (float **)alloc_matrix(num_inputs + 1, num_outputs + 1 )) == 0) {
                        printf("\n Memory allocation error ");
                        exit(1);
    }


        sprintf(state_machine_file_name,"%sstate_machines/state_machine_mem_stochastic.txt",path);
        //sprintf(state_machine_file_name,"/media/alejandro/archivos/MobileRobotSimulator/catkin_ws/src/simulator/src/genetic_behaviors/state_machines/state_machine_mem_stochastic.txt");
        size_mem=read_state_machine_stochastic(state_machine_file_name,mem_state,&num_bits_input,&pr_output);

        flg_read=0;
 }

#ifdef DEBUG
 printf("quantized intensity %d laser %d angle_dest %d\n",q_intensity,obs,dest);
#endif

 //input = (intensity << 4) + (obs << 2) + dest;
 input = (q_intensity << (num_bits_dest_angle+num_bits_vq)) + (obs << num_bits_dest_angle) + dest;

#ifdef DEBUG
 printf("present state %d input %d\n",state,input);
#endif

 index= (((state ) << num_bits_input) + input+1) & 0xffff ;

 *next_state=mem_state[index];
 
 // Function calculate_stochastic_action_output in /robotics/reactive_navigation/reactive_navigation.h
 out=calculate_stochastic_action_output(pr_output, index);

#ifdef DEBUG
 printf("index %d next_state %d stochastic out %d\n",index,*next_state,out);
#endif

if (intensity > THRESHOLD){
            *movements=generate_output(STOP,Mag_Advance,max_angle);
            printf("\n **************** Reached light source ******************************\n");
            result = 1;
                }
else 
 // /home/biorobotica/robotics/utilities/utilities.h //
 *movements=generate_output_genetics(out,Mag_Advance,max_angle);

return result;
}





                 





