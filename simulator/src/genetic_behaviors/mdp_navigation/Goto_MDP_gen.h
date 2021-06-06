/********************************************************************************
*                                                                            	*
*  GoTo_MDP_gen.h 			                              	     	*
*  ===================                                                       	*
*                                                                            	*
*  Description: 							     	*
*  It controls the movement of a robot using MDPs.			     	*
*  It uses a real robot or a simulated one.				     	*
*									     	*
*									     	*
*                               J. Savage, S. Muñoz                           	*
*                                                                        	*
*                               FI-UNAM 2020                                 	*
********************************************************************************/

#define PATH "../data/"
#define CENTROID_FILE "vq_images_laser"
#define NUM_POLICY 4


int Num_backward=0;


// Constants definitions
#define COMMAND_TO_RUN "mdp_stalin/mdpnav -g 1 -e 0.0008 -f "
#define COMMAND_CP "cp policy.txt "

#include "state_machine_mdp.h"
#include "../mdp_stalin/mdpnav_function.h"



// it moves the robot from the origen to the destination using MDP state machines
int go_to_mdp_gen(int num_obs, int dest, int intensity,float *observations, movement *movements, char *root_path, float largest_value,int num_sensors, float Mag_Advance, float max_angle, float intensity_value) 
{

 static int flg=0;
 static int flg_read=1;
 static char last_action=' ';
 static int Time = 1;
 char file_mdp[250];
 float mov_vector_destination;
 float DistTheta;
 static struct mdp_database mdp;
 static int flg_omnidirectional;
 int j;
 int i=0,k;
 int result = 0;
 char command[250];
 static char policy_file[200];
 char action;
 static char previous_action='N';
 char path[250];

 sprintf(path,"%smdp_navigation/",root_path);
 if(flg==0){

 	/* It reads the mdp */
 	strcpy(mdp.objects_file,"mdp_environment");
 	printf("MDPfile %s\n", mdp.objects_file);
 	read_mdps(&mdp,path);

 	flg_omnidirectional=0;
	flg=1;
 }

// It generates the mdp policy
 printf("MDP num_obs %d\n",num_obs);
 if((num_obs == 1) || (Time == NUM_POLICY) || (last_action == 'X')){


		// It generates the ocupancy grid for the mdp, function in file ~/robotics/utilities/utilities.h
 		printf("MDP ocupancy num_obs %d\n",num_obs);
		ocupancy_grid_mdp(observations,num_sensors,dest,&mdp,largest_value);
		write_mdps(mdp,path);

		sprintf(command,"%s%s%smdp_environment_grid.mdp",root_path,COMMAND_TO_RUN,path);	
		printf("COMMAND: %s\n",command);
        	system(command);
        	printf("Done.\n");
    	sprintf(command,"%s%spolicy.txt",COMMAND_CP,path);	
		printf("COMMAND CP: %s\n",command);
        	system(command);
        	printf("Done.\n");

		// it reads the policy table
 		sprintf(policy_file,"%spolicy.txt",path);
 		read_policy(mdp.num_rows,mdp.num_columns,policy_file,&mdp);
 		for(j=1;j<=mdp.num_rows;j++){
    			for(k=1;k<=mdp.num_columns;k++){
                        	printf("policy_table[%d][%d] %c\n",j,k,mdp.policy[j][k]);
    			}
 		}
		flg_read=1;
		Time=1;
        
 }


 // It calculates the robot's movement using a MDP state machine
 action=state_machine_destination_mdp(dest, intensity, path, previous_action,mdp,flg_read);

 flg_read=0;
 printf("Executed action %c\n",action);
 last_action=action;
if (intensity_value > THRESHOLD){
            *movements=generate_output(STOP,Mag_Advance,max_angle);
            printf("\n **************** Reached light source ******************************\n");
            result = 1;
                }
else 
 // /home/biorobotica/robotics/utilities/utilities.h //
 *movements = mdp_output(action, Mag_Advance,max_angle);
 
 previous_action=action;
 Time++;
return result;


}
// it moves the robot from the origen to the destination using MDP state machines
int go_to_mdp_gen_stalin(int num_obs, int dest, int intensity, float *observations, movement *movements,char *root_path,float largest_value,int num_sensors,float Mag_Advance, float max_angle, float intensity_value)
{

 static int flg=0;
 static int flg_read=1;
 static char last_action=' ';
 static int Time = 1;
 char file_mdp[250];
 float mov_vector_destination;
 float DistTheta;
 static struct mdp_database mdp;
 static int flg_omnidirectional;
 int j;
 int i=0,k;
 char command[250];
 static char policy_file[200];
 char action;
 static char previous_action='N';
 float angle_mdp;
 static char mdp_grid_file[250];
 int result = 0;
 char path[200];

  sprintf(path,"%smdp_navigation/",root_path);
 if(flg==0){

    /* It reads the mdp */
    strcpy(mdp.objects_file,"mdp_environment");
    printf("MDPfile %s\n", mdp.objects_file);
    read_mdps(&mdp,path);

    flg_omnidirectional=0;
    flg=1;
 }

// It generates the mdp policy
 //printf("MDP num_obs %d\n",num_obs);
 if((num_obs == 1) || (Time == NUM_POLICY) || (last_action == 'X')){


                // It generates the ocupancy grid for the mdp, function in file ~/robotics/utilities/utilities.h
                //printf("MDP ocupancy num_obs %d\n",num_obs);
                ocupancy_grid_mdp(observations,num_sensors,dest,&mdp,largest_value);
                write_mdps(mdp,path);

        // mdpnav -g 1 -e 0.0008 -f 
        /*
                sprintf(command,"%s%smdp_environment_grid.mdp",COMMAND_TO_RUN,inputs.path);
                printf("COMMAND: %s\n",command);
                system(command);
                printf("Done.\n");
                sprintf(command,"%s%spolicy.txt",COMMAND_CP,inputs.path);
                printf("COMMAND CP: %s\n",command);
                system(command);
                printf("Done.\n");
        */

        //mdpnav_function ../mdp_stalin/mdpnav_function.h
        sprintf(mdp_grid_file,"%smdp_environment_grid.mdp",path);
        sprintf(policy_file,"%spolicy.txt",path);
        mdpnav_function(mdp, mdp_grid_file, policy_file);

                // it reads the policy table
                //sprintf(policy_file,"%spolicy.txt",inputs.path);
        // function read_policy in state_machine_mdp.h
                read_policy(mdp.num_rows,mdp.num_columns,policy_file,&mdp);
#ifdef DEBUG
                for(j=1;j<=mdp.num_rows;j++){
                        for(k=1;k<=mdp.num_columns;k++){
                                printf("policy_table[%d][%d] %c\n",j,k,mdp.policy[j][k]);
                        }
                }
#endif
                flg_read=1;
                Time=1;
 }


 // It calculates the robot's movement using a MDP state machine
 action=state_machine_destination_mdp(dest, intensity, path, previous_action,mdp,flg_read);

 flg_read=0;
 printf("Executed action %c\n",action);
 last_action=action;
if (intensity_value > THRESHOLD){
            *movements=generate_output(STOP,Mag_Advance,max_angle);
            printf("\n **************** Reached light source ******************************\n");
            result = 1;
                }
else 
 // /home/biorobotica/robotics/utilities/utilities.h //
 *movements = mdp_output(action, Mag_Advance,max_angle);
 
 previous_action=action;
 Time++;
return result;

}


