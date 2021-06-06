/********************************************************
 *                                                      *
 *                                                      *
 *      state_machine_hmm.h                             *
 *                                                      *
 *		Jesus Savage				*
 *		FI-UNAM					*
 *		5-5-2013                                *
 *                                                      *
 ********************************************************/


#define HMM_FSM "hmm_fsm"
#include "hmm_lib.h"

#define NUM_INPUTS_DESTINATION 4
//#define NUM_MAX_SENSORS 4097
#define MAG_ADVANCE 0.04 // Advance magnitude in meters




// it converts a binary output to a robot's output
Behavior hmm_symbol_output(int out, float advance, float angle){

  Behavior output;
  //float angle=(float) (PI/(4.0*15.0));
  //float advance;
 // int bits_mag,mag,bits_dir,dir;
  //float mag_angle;
  

  //advance= MAG_ADVANCE*1.00;  
 // mag=1;
  //mag_angle=8.0;


  switch(out){

	case 0: // Stop
                output.xc=0.0f;
                output.yc=0.0f;
                printf("Action STOP\n");
                break;

        case 1: // Forward
                output.xc=advance;
                output.yc=0.0f;
                printf("Action FORWARD\n");
                break;

        case 2: // backward
                output.xc=-advance;
                output.yc=0.0f;
                //Num_backward++;
                printf("Action BACKWARD\n");
                break;

        case 3:// Turn left
                output.xc=0.0f;
                output.yc=angle;
                printf("Action LEFT\n");
                break;

        case 4: // Turn right
                output.xc=0.0f;
                output.yc=-angle;
                printf("Action RIGHT\n");
                break;

        default:printf("Output %d not defined used STOP\n", out);
                output.xc=0;
                output.yc=0;
                break;
  }

  return(output);

}
                             



// Engine to execute probabilistic state machines 
int state_machine_destination_hmm(int dest, int q_intensity, int obs, int *next_state,float Mag_Advance,
				float max_angle, char *individual, char *path, int num_bits_vq, movement *movements, float intensity){

 //AdvanceAngle gen_vector;

 static int states[NUM_MAX_BEHAVIORS*4];


 static int previous_state=1;
 static int flg_old=1;
 //Behavior output;
 int out_symbol;
 static int flg_read=1;
 static struct hmm_database hmm;
 static int num_states,num_inputs,num_outputs;
 static int Time;
 int quantized_value;
 int flg_inputs=1; // it uses the inputs to calculate the outputs
 int num_bits_intensity = NUM_BITS_INTENSITY;
 int num_bits_dest_angle = NUM_BITS_DEST_ANGLE;
 int result = 0;



#ifdef DEBUG
 printf("\n\n *********************state machine HMM**********\n");
 printf("individual %s\n",individual);
 printf(" obs intensity %d dest %d\n",q_intensity,dest);
#endif

 /* 
 if(reset==1){
        state[1]=previous_state;
        num_steps=0;
 }
 */


 if(flg_read==1){

        /* It reads the number of states, num. obs. symbols and nun. out symbols of the hmm */
	strcpy(hmm.objects_file,individual);
#ifdef DEBUG
        printf("HMM file %s\n", hmm.objects_file);
#endif

	// finction read_hmms in hmm_lib.h
        read_hmms(&hmm,0,path,flg_inputs);

	//It creates the HMM structure
        hmm_initialize(&hmm);

        /* It reads the complete hmm */
        read_hmms(&hmm,1,path,flg_inputs);
	num_states=hmm.num_states;
        num_inputs=hmm.num_obs_symbols;
   	num_outputs=hmm.num_out_symbols;
	
	
	// Initializes the random generator
        srandom((int)time((time_t *)NULL));

	states[Time]=calculate_hmm_first_state(hmm);

	flg_read=0;

 }


 // It joints the obstacles and destination quantized sensors
 quantized_value = (q_intensity << (num_bits_dest_angle+num_bits_vq)) + (obs << num_bits_dest_angle) + dest;
#ifdef DEBUG
 printf("Observation %d\n", quantized_value);
#endif


 // it calculates the output
 if(flg_old == 0){

 	out_symbol=calculate_hmm_output(hmm,states[Time],quantized_value,flg_inputs);
 	//output=hmm_symbol_output(out_symbol,Mag_Advance,max_angle);
if (intensity > THRESHOLD){
            *movements=generate_output(STOP,Mag_Advance,max_angle);
            printf("\n **************** Reached light source ******************************\n");
            result = 1;
                }
else
	// /home/biorobotica/robotics/utilities/utilities.h //
 	*movements=generate_output_genetics(out_symbol,Mag_Advance,max_angle);
 	//gen_vector.distance=output.xc;
 	//gen_vector.angle=output.yc;

#ifdef DEBUG
 	printf("HMM present State[%d] %d \n",Time, states[Time]);
#endif

	 // it calculates the next state
 	*next_state=get_next_state_hmm_viterbi(quantized_value,flg_old, hmm,states[Time]);
#ifdef DEBUG
 	printf("Viterbi next  State %d \n",*next_state);
#endif
 	//next_state=get_next_state_hmm(quantized_value,flg_old, hmm,state[Time]);
 	//printf("Calculate  State %d \n",next_state);

 	Time++;
 	//next_state=calculate_hmm_next_state(hmm,state[Time],quantized_value);

 	states[Time]=*next_state;
 	previous_state=states[Time];

 }
 else{
	// it generates no output in the first time
	//gen_vector.distance= 0.0;
	//gen_vector.angle= 0.0;
	*movements=generate_output(STOP,Mag_Advance,max_angle);
	Time=1;
 	*next_state=get_next_state_hmm_viterbi(quantized_value,flg_old, hmm,states[Time]);
#ifdef DEBUG
 	printf("First  State %d \n",*next_state);
#endif
 	states[Time]=*next_state;
 	flg_old=0;
  }

 return result;


}



