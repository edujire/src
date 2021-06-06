/********************************************************
 *                                                      *
 *                                                      *
 *      state_machine_mdp.h                             *
 *                                                      *
 *		Jesus Savage				*
 *		FI-UNAM					*
 *		7-1-2019                                *
 *                                                      *
 ********************************************************/



#define NUM_INPUTS_DESTINATION 4
//#define NUM_MAX_SENSORS 4097
#define MAG_ADVANCE 0.04 // Advance magnitude in meters
//#define MAX_TYPE 20
//#define ROW_PRB 3
//#define CLM_PRB 3
#define FIRST_STATE_I 3
#define FIRST_STATE_J 3 
#define LIMIT_I 6
#define LIMIT_J 6

/*
struct mdp_database {
        char objects_file[300];
	int type[MAX_TYPE][MAX_TYPE];
	float reward[MAX_TYPE][MAX_TYPE];
	char policy[MAX_TYPE][MAX_TYPE];
	float pr[9][ROW_PRB+1][CLM_PRB+1];
        int num_rows;
        int num_columns;
};

struct table {
 int x[4][4];
 int y[4][4];
};
*/

/* Function that calculates the MDP output */
char calculate_mdp_action(struct mdp_database mdp, char action)
{
 
 int index=1;
 float rand;
 int i,j,k;
 int jj,kk;
 float tmp;

 char outputs[4][4][1];

 //Notation North=N, North-East=A, East=E, South-East=I, South=S, South-West=O, West=W, North-West=U, Terminal=X
 outputs[1][1][1]='U';
 outputs[1][2][1]='N';
 outputs[1][3][1]='A';
 outputs[2][1][1]='W';
 outputs[2][2][1]='C';
 outputs[2][3][1]='E';
 outputs[3][1][1]='O';
 outputs[3][2][1]='S';
 outputs[3][3][1]='I';

 printf("Possible Action ");

 switch(action){

        case 'N': // North
                printf("North\n");
		index=1;
                break;

        case 'A': // North-East
                printf("North-East\n");
		index=2;
                break;

        case 'E': // East
                printf("East\n");
		index=3;
                break;

        case 'I': // South-East
                printf("South-East\n");
		index=4;
                break;

        case 'S': // South
                printf("South\n");
		index=5;
                break;

        case 'O': // South-West
                printf("South-West\n");
		index=6;
                break;

        case 'W': // West
                printf("West\n");
		index=7;
                break;

        case 'U': // North-West
                printf("North-West\n");
		index=8;
                break;

        case 'X': // Terminal
                printf("Terminal\n");
		return 'X';
                break;

	default:printf("Output %c not defined used STOP\n", action);
                break;
  }


 // it generates a Uniform random variable between 0 and 1
 rand=generaR(0.0,1.0);
 //printf("random %f index %d\n",rand,index);

 tmp=0.0;

 jj=ROW_PRB;
 kk=CLM_PRB;
 for(j=1;j<=ROW_PRB;j++){
 	for(k=1;k<=CLM_PRB;k++){
    	        //printf("acumulated_pr[%d][%d] %f\n",j,k,tmp);
               	tmp = tmp + mdp.pr[index][j][k];
        	if(tmp >= rand){
                	jj=j;
			kk=k;
			j=ROW_PRB+1;
			k=CLM_PRB+1;
        	}
   }

 }
   
 //printf("Notation North=N, North-East=A, East=E, South-East=I, South=S, South-West=O, West=W, North-West=U, Terminal=X\n");
 //printf("stochastic action %d %d acumulated %f\n",jj,kk,tmp);
 //printf("outputs[%d][%d] %c\n",jj,kk,outputs[jj][kk][1]);

 return outputs[jj][kk][1];

}





// it converts an action to a robot's movement
movement mdp_output(char action, float advance, float angle){

 movement output;
 
 //printf("Most Probable ");

 switch(action){

        case 'N': // North
		output.advance=advance;
                //output.angle=2*angle - angle_robot;
                output.twist=2*angle;
                //printf("Action North\n");
                break;

        case 'A': // North-East
		output.advance=advance;
                //output.angle=angle - angle_robot;
                output.twist=angle;
                //printf("Action North-East\n");
                break;

        case 'E': // East
		output.advance=advance;
                //output.angle=0.0 - angle_robot;
                output.twist=0.0;
                //printf("Action East\n");
                break;

        case 'I': // South-East
		output.advance=advance;
                //output.angle=-angle - angle_robot;
                output.twist=-angle;
                //printf("Action South-East\n");
                break;

        case 'S': // South
		output.advance=advance;
                //output.angle=-2*angle - angle_robot;
                output.twist=-2*angle;
                //printf("Action South\n");
                break;

        case 'O': // South-West
		output.advance=advance;
                //output.angle=-angle*3.0 - angle_robot;
                output.twist=-angle*3.0;
                //printf("Action South-West\n");
                break;

        case 'W': // West
		output.advance=-advance;
                //output.angle=0.0 - angle_robot;
                output.twist=0.0;
                //printf("Action West\n");
                break;

        case 'U': // North-West
		output.advance=advance;
                //output.angle=3*angle - angle_robot;
                output.twist=3*angle;
                //printf("North-West\n");
                break;

        case 'X': // Terminal
		output.advance=0.0;
                output.twist=0.0;
                //printf("Terminal\n");
                break;

        default:
		//printf("Output %c not defined used STOP\n", action);
		output.advance=0.0;
                output.twist=0.0;
                break;
  }

  return(output);

}


// Function that reads the policy table
void  read_policy(int num_rows, int num_columns, char *policy_file, struct mdp_database *mdp)
{

 FILE *fp;
 int j,k;
 char c_tmp;
 int jj;


#ifdef DEBUG
 printf("Reading Policy file: %s\n",policy_file);
#endif
 fp=fopen(policy_file,"r");
 if(fp==0){
        printf("\n File %s does not exists",policy_file);
        exit(0);
 }

 for(j=1,jj=num_rows;j<=num_rows;j++,jj--){
        for(k=1;k<=num_columns;k++){
                fscanf(fp,"%c",&c_tmp);
		//printf("text %x\n",c_tmp);
                mdp->policy[jj][k]=c_tmp;
                //fscanf(fp,"%c",&c_tmp); // space reading
                //printf("policy_table[%d][%d] %c\n",jj,k,mdp->policy[jj][k]);
        }
        fscanf(fp,"%c",&c_tmp); // carry return reading
 }


}

                             

/* Function that reads the mdp*/
void read_mdps(struct mdp_database *mdp, char *path)
{
   FILE *fp2;
   float average,tmp;
   int j,k,int_tmp,jj;
   char input_file[300];
   int n_rows,n_columns;


   /* It opens the mdp file */
   sprintf(input_file,"%s%s.mdp",path,mdp->objects_file);
#ifdef DEBUG
   printf("Reading MDP FILE: %s\n",input_file);
#endif
   fp2=fopen(input_file,"r");
   if(fp2==0){
        printf("\n File %s does not exists",input_file);
        exit(0);
   }

   /* It reads the number of rows and columns */
   fscanf(fp2,"%d",&n_rows);
   mdp->num_rows=n_rows;
   fscanf(fp2,"%d",&n_columns);
   mdp->num_columns=n_columns;
#ifdef DEBUG
   printf("Num.rows %d Num.columns %d \n",n_rows,n_columns);
#endif


   if((n_rows>=MAX_TYPE) || (n_columns>=MAX_TYPE)) {

	printf("Increase constant MAX_TYPE on file ~/robotics/mdp_navigation/state_machine_mdp.h\n");
	exit(0);

   }


   // It reads the type and the rewards of the MDP celds 
   for(j=1,jj=n_rows;j<=n_rows;j++,jj--){
        for(k=1;k<=n_columns;k++){
                fscanf(fp2,"%d",&int_tmp);
                mdp->type[jj][k]=int_tmp;
                //printf("type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
                fscanf(fp2,"%f",&tmp);
                mdp->reward[jj][k]=tmp;
                //printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
        }
   }


   // It reads the north probabilities 
   average=0.0;
   for(j=1;j<=ROW_PRB;j++){
        for(k=1;k<=CLM_PRB;k++){
                fscanf(fp2,"%f",&tmp);
                mdp->pr[1][j][k]=tmp;
#ifdef DEBUG
                printf("north[%d][%d] %f\n",j,k,mdp->pr[1][j][k]);
#endif
		average+=tmp;
        }
   }
#ifdef DEBUG
   printf("average north %f\n", average);
#endif
   // It reads the north-east probabilities 
   average=0.0;
   for(j=1;j<=ROW_PRB;j++){
        for(k=1;k<=CLM_PRB;k++){
                fscanf(fp2,"%f",&tmp);
                mdp->pr[2][j][k]=tmp;
#ifdef DEBUG
                printf("north_east[%d][%d] %f\n",j,k,mdp->pr[2][j][k]);
#endif
                average+=tmp;
        }
   }
#ifdef DEBUG
   printf("average north_east %f\n", average);
#endif

  // It reads the east probabilities 
   average=0.0;
   for(j=1;j<=ROW_PRB;j++){
        for(k=1;k<=CLM_PRB;k++){
                fscanf(fp2,"%f",&tmp);
                mdp->pr[3][j][k]=tmp;
#ifdef DEBUG
                printf("east[%d][%d] %f\n",j,k,mdp->pr[3][j][k]);
#endif
                average+=tmp;
        }
   }
#ifdef DEBUG
   printf("average east %f\n", average);
#endif

 // It reads the south-east probabilities 
   average=0.0;
   for(j=1;j<=ROW_PRB;j++){
        for(k=1;k<=CLM_PRB;k++){
                fscanf(fp2,"%f",&tmp);
                mdp->pr[4][j][k]=tmp;
#ifdef DEBUG
                printf("south-east[%d][%d] %f\n",j,k,mdp->pr[4][j][k]);
#endif
                average+=tmp;
        }
   }
#ifdef DEBUG
   printf("average south-east %f\n", average);
#endif
 // It reads the south probabilities 
   average=0.0;
   for(j=1;j<=ROW_PRB;j++){
        for(k=1;k<=CLM_PRB;k++){
                fscanf(fp2,"%f",&tmp);
                mdp->pr[5][j][k]=tmp;
#ifdef DEBUG
                printf("south[%d][%d] %f\n",j,k,mdp->pr[5][j][k]);
#endif
                average+=tmp;
        }
   }
#ifdef DEBUG
   printf("average south %f\n", average);
#endif

 // It reads the south-west probabilities 
   average=0.0;
   for(j=1;j<=ROW_PRB;j++){
        for(k=1;k<=CLM_PRB;k++){
                fscanf(fp2,"%f",&tmp);
                mdp->pr[6][j][k]=tmp;
#ifdef DEBUG
                printf("south-west[%d][%d] %f\n",j,k,mdp->pr[6][j][k]);
#endif
                average+=tmp;
        }
   }
#ifdef DEBUG
   printf("average south-west %f\n", average);
#endif
// It reads the west probabilities 
   average=0.0;
   for(j=1;j<=ROW_PRB;j++){
        for(k=1;k<=CLM_PRB;k++){
                fscanf(fp2,"%f",&tmp);
                mdp->pr[7][j][k]=tmp;
#ifdef DEBUG
                printf("west[%d][%d] %f\n",j,k,mdp->pr[7][j][k]);
#endif
                average+=tmp;
        }
   }
#ifdef DEBUG
   printf("average west %f\n", average);
#endif

// It reads the north-west probabilities 
   average=0.0;
   for(j=1;j<=ROW_PRB;j++){
        for(k=1;k<=CLM_PRB;k++){
                fscanf(fp2,"%f",&tmp);
                mdp->pr[8][j][k]=tmp;
#ifdef DEBUG
                printf("north-west[%d][%d] %f\n",j,k,mdp->pr[8][j][k]);
#endif
                average+=tmp;
        }
   }
#ifdef DEBUG
   printf("average west %f\n", average);
#endif
 fclose(fp2);

}



void calculate_next_state(int si,int sj,char action,int *i,int *j){


 switch(action){

        case 'N': // North
                //printf("North\n");
                *i=si+1;
		*j=sj;
                break;

        case 'A': // North-East
                //printf("North-East\n");
                *i=si+1;
		*j=sj+1;
                break;

        case 'E': // East
                *i=si;
		*j=sj+1;
                //printf("East\n");
                break;

        case 'I': // South-East
                *i=si-1;
		*j=sj+1;
                //printf("South-East\n");
                break;

        case 'S': // South
                *i=si-1;
		*j=sj;
                //printf("South\n");
                break;

        case 'O': // South-West
                *i=si-1;
		*j=sj-1;
                //printf("South-West\n");
                break;

        case 'W': // West
                *i=si;
		*j=sj-1;
                //printf("South-West\n");
                break;

        case 'U': // North-West
                *i=si+1;
		*j=sj-1;
                //printf("SM North-West %d %d\n",*i,*j);
                break;

        case 'X': // Terminal
                //printf("Terminal\n");
                *i=si;
		*j=sj;
                break;

        default:
		//printf("Output %c not defined used STOP\n", action);
                break;
 }

 if(*i > LIMIT_I)*i=LIMIT_I;
 else if (*i < 1)*i=1;

 if(*j > LIMIT_J)*j=LIMIT_J;
 else if (*j < 1)*j=1;

}





// Engine to execute mdp state machine 
char state_machine_destination_mdp(int dest, int intensity, char *path, char executed_action, struct mdp_database mdp, int flg){

 AdvanceAngle gen_vector;
 int i,j,k;
 static int policy_table[MAX_TYPE][MAX_TYPE];
 static int previous_state=1;
 static int flg_old=1;
 Behavior output;
 int out_symbol;
 static int num_states,num_inputs,num_outputs;
 static int Time;
 int quantized_value;
 static int states_i[100],states_j[100];
 char action,possible_action;


 //printf("\n\n *********************state machine MDP**********\n");
 //printf(" obs intensity %d dest %d\n",intensity,dest);


 if(flg == 1){
        // it gets the first state
        Time=1;
        states_i[Time]=FIRST_STATE_I;
	i=FIRST_STATE_I;
        states_j[Time]=FIRST_STATE_J;
	j=FIRST_STATE_J;
	// Initializes the random generator
 }
 else {
 	calculate_next_state(states_i[Time],states_j[Time],executed_action,&i,&j);
	Time++;
	states_i[Time]=i;
	states_j[Time]=j;
 }

#ifdef DEBUG
 printf("\n ***** %d State %d %d *****\n",Time,states_i[Time],states_j[Time]);
#endif
 possible_action=mdp.policy[i][j];
#ifdef DEBUG
 printf("Possible ACTION %c\n",possible_action);
#endif


 // it generates an stochastic output
 /*action=calculate_mdp_action(mdp, possible_action);
 printf("Most probable ACTION %c\n",action);
 return action;
 */

 return possible_action;

}



