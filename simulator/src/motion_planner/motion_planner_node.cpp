/***********************************************
*                                              *
*      motion_planner_node.cpp                 *
*                                              *
*      Jesus Savage                            *
*      Diego Cordero                           *
*                                              *
*              Bio-Robotics Laboratory         *
*              UNAM, 17-2-2020                 *
*                                              *
*                                              *
************************************************/
#define CENTROID_FILE "data/vq_images_laser"
#define NUM_BITS_INPUTS 8
#define NUM_BITS_OUTPUTS 3 // bits to decode 8 outputs: STOP, BACKWARD, FORWARD, TURN_LEFT, TURN_RIGHT, etc
#define NUM_BITS_INTENSITY 2
#define NUM_INTENSITY 4 // 2 ** NUM_BITS_INTENSITY
#define NUM_BITS_DEST_ANGLE 3
#define NUM_DEST_ANGLE 8 // 2 
#define NUM_MAX_MEMORY 65535 // 2 >> 16
//incomplete
#define LARGEST_DISTANCE_SENSORS 5.0 // 5.0 meters

#include <stdio.h>
#include <math.h>

#include "ros/ros.h"
#include "../utilities/simulator_structures.h"
#include "../utilities/random.h"
#include "motion_planner_utilities.h"
#include "../state_machines/light_follower.h"
#include "../state_machines/sm_avoidance.h"
#include "../state_machines/sm_avoidance_destination.h"
#include "../state_machines/sm_destination.h"
#include "../state_machines/user_sm.h"
#include "../state_machines/dijkstra.h"
#include "../state_machines/dfs.h"
#include "clips_ros/SimuladorRepresentation.h"
#include "../behaviors/oracle.h"
#include "../action_planner/action_planner.h"
//Genetics
//#include "../genetic_behaviors/utilities/constants.h"
#include "../genetic_behaviors/utilities/structures.h"
#include "../genetic_behaviors/vq/vq.h"
#include "../genetic_behaviors/utilities/utilities.h"
//#include "../genetic_behaviors/simulator/simulation.h"
//#include "../genetic_behaviors/utilities/inputs.h"
#include "../genetic_behaviors/reactive_navigation/reactive_navigation.h"
#include "../genetic_behaviors/state_machines/state_machine_engine.h"
//#include "../genetic_behaviors/utilities/random.h"
#include "../genetic_behaviors//hmm_robots/state_machine_hmm.h"
#include "../genetic_behaviors/mdp_navigation/Goto_MDP_gen.h"
//#include "../genetic_behaviors/mdp_stalin/mdpnav_function.h"

float K_INTENSITY;

int main(int argc ,char **argv)
{
    ros::init(argc ,argv ,"simulator_motion_planner_node");
    ros::NodeHandle n;
    ros::Subscriber params_sub = n.subscribe("simulator_parameters_pub",0, parametersCallback);
    ros::Subscriber sub = n.subscribe("/scan", 10, laserCallback);
    SimuladorRepresentation::setNodeHandle(&n);
    ros::Rate r(20);


    float lidar_readings[512];
    float light_readings[8];
    
    int i;
    int tmp;
    int sensor;
    int est_sig;
    int q_light;
    int q_inputs;
    int flagOnce;
    int flg_finish;
    int mini_sm=1;
    int cta_steps;
    int flg_result;
    int flg_noise=0;
    
    float result;
    float final_x;
    float final_y;
    float intensity;
    float max_advance;
    float max_turn_angle;
    float noise_advance;
    float noise_angle;
    
    char path[100];
    char object_name[20];


    movement movements;
    step steps[200];
    step graph_steps[200];

   //genetic behaviors
   float scale = 1.0; 
   int num_bits_vq = 3;
   int size_vq = pow(2,num_bits_vq);
   int q_intensity;
   int q_inputs_gen;
   int q_light_gen;
   int flagRun=0;
   char genetic_path[200]; //path of data used for genetic behaviors
   char pkg_path[100];
   char file_obs[250]; //file of simulation steps to get individual fitness
   FILE *fpw;
   strcpy(pkg_path,ros::package::getPath("simulator").c_str());
   sprintf(genetic_path,"%s/src/genetic_behaviors/",pkg_path);
    printf("genetics path: %s \n",genetic_path);
    int inner_state = 0; //to give one step and one pause
    // it sets the environment's path
    strcpy(path,"./src/simulator/src/data/");
    //HMM
    char file_hmm[250];
    strcpy(file_hmm,"hmm_robots/hmm_destination_avoidance_fsm.prb");
    //MDP
    float angle_mdp;
    int num_obs = 1;
    int rotate = 1;
    char object_file[200];

    //real robot
    int flg_stop = 0;

    //test
    FILE *test;
    char file_test[200];
    float y;
    float y_hat;
    float front;
    float back;
    static int NUM_RUN=0;
    while( ros::ok()  )
    {
        flagOnce = 1;
        cta_steps = 0;
        mini_sm =1;

        while( params.run )
        {
            // it gets sensory data
            ros::spinOnce();

            if (!params.useRealRobot)
            {
                get_light_values(&intensity,light_readings); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h

                get_lidar_values(lidar_readings,params.robot_x,
                                 params.robot_y,params.robot_theta,params.useRealRobot); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h
            }
            else
            {
                get_light_values_RealRobot(&intensity,light_readings); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h
                for( i = 0; i < 512; i++){
                    //if (lasers[i] > 0.38) lidar_readings[i] = 0.30;
                    //else (lasers[i] > 0.38) lidar_readings[i] = (lasers[i]-0.08); //minus offset of readings
                    if (lasers[i] > 0.38) lidar_readings[i] = 0.30;
                    else lidar_readings[i] = (lasers[i]-0.08); 
                }
                scale = 0.5;
                params.laser_value = 0.30;
            }

            // it quantizes the sensory data
            q_light = quantize_light(light_readings); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h
            
            if(params.noise )
                q_inputs = quantize_laser_noise(lidar_readings,params.laser_num_sensors,params.laser_value); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h
            else
            	if (params.useRealRobot)
            		q_inputs = quantize_laser_real(lidar_readings,params.laser_num_sensors,params.laser_value);
            	else
                	q_inputs = quantize_laser(lidar_readings,params.laser_num_sensors,params.laser_value); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h


            max_advance = params.robot_max_advance;
            max_turn_angle = params.robot_turn_angle;
            //genetic behavior quantized variables
            K_INTENSITY=.2*max_advance;
            q_intensity = quantize_intensity(intensity);
            q_light_gen = quantize_destination(light_readings);
            q_inputs_gen = quantize_inputs(lidar_readings, params.laser_num_sensors, size_vq, genetic_path, scale); // in genetic_behaviors/utilities/utilities.h
            //saves robot info
            if(flagOnce==1){
                sprintf(file_obs,"%sdata/output.raw",genetic_path);     
                if((fpw=fopen(file_obs,"w")) == NULL){
                    printf("File %s can not be open\n",file_obs);
                    return(0);
                }
                sprintf(file_test,"%sdata/test.raw",genetic_path);
                if((test=fopen(file_test,"w")) == NULL){
                    printf("File %s can not be open\n",file_test);
                    return(0);
                }
                // it opens the observation's sensor file
                fprintf(fpw,"( radio_robot %f )\n",params.robot_radio);
                fprintf(fpw,"( origen %f %f %f )\n",params.robot_x,params.robot_y,params.robot_theta);
                fprintf(fpw,"( destination %f %f )\n",params.light_x,params.light_y);
                flagRun=1;
            }
            fprintf(fpw,"( robot Justina %f %f %f )\n",params.robot_x,params.robot_y,params.robot_theta);
            write_obs_sensor(fpw,lidar_readings,params.laser_num_sensors,params.laser_origin,params.laser_range);
            fprintf(fpw,"( sensor destination %d )\n",q_light_gen);
            fprintf(fpw,"( sensor light %d )\n",q_intensity);
            fprintf(fpw,"( intensity %f )\n",intensity);


            switch ( params.behavior)
            {

            case 1:
                // This function sends light sensory data to a function that follows a light source and it issues
                // the actions that the robot needs to perfom.
                // It is located in ~/catkin_ws/src/simulator/src/state_machines/light_follower.h
                flg_result = light_follower(intensity, light_readings,&movements,max_advance,max_turn_angle);
                if(flg_result == 1) flg_stop=1;
                break;

            case 2:
                // This function sends light sensory data to an state machine that follows a light source and it issues
                // the actions that the robot needs to perfom.
                // It is located in ~/catkin_ws/src/simulator/src/state_machines/sm_destination.h
                if(flagOnce)
                {
                    est_sig = 1;
                    flagOnce = 0;
                }
                flg_result = sm_destination(intensity,q_light,&movements,&est_sig,params.robot_max_advance,params.robot_turn_angle);
                if(flg_result == 1) flg_stop=1;

                break;

            case 3:
                // This function sends quantized sensory data to an state machine that avoids obstacles and it issues
                // the actions that the robot needs to perfom.
                // It is located in ~/catkin_ws/src/simulator/src/state_machines/sm_avoidance.h
                if(flagOnce)
                {
                    est_sig = 0;
                    flagOnce = 0;
                }
                sm_avoid_obstacles(q_inputs,&movements,&est_sig ,params.robot_max_advance ,params.robot_turn_angle);
                break;

            case 4:
                // This function sends quantized sensory data to an state machine that follows a light source and avoids obstacles
                // and it issues the actions that the robot needs to perfom.
                // It is located in ~/catkin_ws/src/simulator/src/state_machines/sm_avoidance_destination.h
                if(flagOnce)
                {
                    est_sig = 0;
                    flagOnce = 0;
                }
                if(params.useRealRobot){
                    if(inner_state == 0){
                        flg_result=sm_avoidance_destination(intensity,q_light_gen,q_inputs,&movements,&est_sig,
                                                    params.robot_max_advance ,params.robot_turn_angle);
                        inner_state ==1;
                    }
                    else{
                       movements.twist=0.0;
                        movements.advance =0.0;
                        inner_state == 0; 
                    }
                }
                else
                flg_result=sm_avoidance_destination(intensity,q_light,q_inputs,&movements,&est_sig,
                                                    params.robot_max_advance ,params.robot_turn_angle);

                if(flg_result == 1) flg_stop=1;
                break;

            case 5:
                // This function sends the intensity and the quantized sensory data to a Clips node and it receives the actions that
                // the robot needs to perfom to avoid obstacles and reach a light source according to first order logic
                // It is located in ~/catkin_ws/src/simulator/src/behaviors/oracle.h
                result=oracle_clips(intensity,q_light,q_inputs,&movements,max_advance ,max_turn_angle);
                if(result == 1.0) flg_stop=1;
                break;


            case 6:
                // it finds a path from the origen to a destination using depth first search
                if(flagOnce)
                {
                    for(i = 0; i < 200; i++) steps[i].node = -1;

                    // it finds a path from the origen to a destination using first search
                    dfs(params.robot_x ,params.robot_y ,params.light_x ,params.light_y ,params.world_name,steps);
                    print_algorithm_graph (steps);
                    i = 0;
                    final_x = params.light_x;
                    final_y = params.light_y;
                    set_light_position(steps[i].x,steps[i].y);
                    printf("New Light %d: x = %f  y = %f \n",i,steps[i].x,steps[i].y);
                    flagOnce = 0;
                    flg_finish=0;
                    est_sig = 0;
                    movements.twist=0.0;
                    movements.advance =0.0;
                }
                else
                {
                    //flg_result=sm_avoidance_destination(intensity,q_light,q_inputs,&movements,&est_sig,                                                        //params.robot_max_advance ,params.robot_turn_angle);
                    flg_result = oracle_clips(intensity,q_light,q_inputs,&movements,max_advance ,max_turn_angle);

                    if(flg_result == 1)
                    {
                        if(flg_finish == 1)
                            flg_stop=1;
                        else
                        {
                            if(steps[i].node != -1)
                            {
                                set_light_position(steps[i].x,steps[i].y);
                                printf("New Light %d: x = %f  y = %f \n",i,steps[i].x,steps[i].y);
                                printf("Node %d\n",steps[i].node);
                                i++;
                                //printf("type a number \n");
                                //scanf("%d",&tmp);
                            }
                            else
                            {
                                set_light_position(final_x,final_y);
                                printf("New Light %d: x = %f  y = %f \n",i,final_x,final_y);
                                flg_finish = 1;
                            }
                        }
                    }
                }

                break;

            case 7:
                if(flagOnce)
                {
                    for(i = 0; i < 200; i++)steps[i].node=-1;
                    // it finds a path from the origen to a destination using the Dijkstra algorithm
                    dijkstra(params.robot_x ,params.robot_y ,params.light_x ,params.light_y ,params.world_name,steps);
                    print_algorithm_graph (steps);
                    i=0;
                    final_x=params.light_x;
                    final_y= params.light_y;
                    set_light_position(steps[i].x,steps[i].y);
                    printf("New Light %d: x = %f  y = %f \n",i,steps[i].x,steps[i].y);
                    flagOnce = 0;
                    flg_finish=0;
                    est_sig = 0;
                    movements.twist=0.0;
                    movements.advance =0.0;
                }
                else
                {
                    flg_result=sm_avoidance_destination(intensity,q_light,q_inputs,&movements,&est_sig,
                                                        params.robot_max_advance ,params.robot_turn_angle);

                    if(flg_result == 1)
                    {
                        if(flg_finish == 1) flg_stop=1;
                        else
                        {
                            if(steps[i].node != -1)
                            {
                                set_light_position(steps[i].x,steps[i].y);
                                printf("New Light %d: x = %f  y = %f \n",i,steps[i].x,steps[i].y);
                                printf("Node %d\n",steps[i].node);
                                i++;
                                //printf("type a number \n");
                                //scanf("%d",&tmp);
                            }
                            else
                            {
                                set_light_position(final_x,final_y);
                                printf("New Light %d: x = %f  y = %f \n",i,final_x,final_y);
                                flg_finish=1;
                            }
                        }
                    }
                }
                break;

            case 8:
                // Here it goes your code for selection 8
                if(flagOnce)
                {
                    est_sig = 0;
                    flagOnce = 0;
                }
                user_sm(intensity,light_readings, lidar_readings, params.laser_num_sensors,params.laser_value,
                        q_light,q_inputs,&movements,&est_sig ,params.robot_max_advance ,params.robot_turn_angle);
                break;

            case 9:

                flg_result=light_follower(intensity, light_readings,&movements,max_advance,max_turn_angle);
                if(flg_result == 1)
                    set_light_position(.5,.25);

                break;

            
            case 10:

		action_planner(params.robot_x, params.robot_y,params.robot_theta,&movements);

                break;
            //DERIVED FSM
            case 11:

            if(flagOnce)
            {
               est_sig = 0;
               flagOnce = 0;
            }
            //printf("K_INTENSITY: %f,  q_intensity: %d",K_INTENSITY,q_intensity);
            if(params.useRealRobot){
                    if(inner_state == 0){
                        flg_result = state_machine_engine(q_inputs_gen, q_light_gen, q_intensity, &movements, &est_sig, max_advance, max_turn_angle,
                genetic_path, num_bits_vq, intensity,params.flg_genetics,params.individuo, params.file_behavior);

                        inner_state ==1;
                    }
                    else{
                       movements.twist=0.0;
                        movements.advance =0.0;
                        inner_state == 0; 
                    }
                }
                else
                flg_result = state_machine_engine(q_inputs_gen, q_light_gen, q_intensity, &movements, &est_sig, max_advance, max_turn_angle,
                genetic_path, num_bits_vq, intensity,params.flg_genetics,params.individuo, params.file_behavior);

            

            if(flg_result == 1) flg_stop=1;
            break;

            //DERIVED FSM STOCHASTIC
            case 12:

            if(flagOnce)
            {
               est_sig = 0;
               flagOnce = 0;
            }
            if(params.useRealRobot){
                if(inner_state == 0){
                    flg_result = state_machine_engine_stochastic(q_inputs_gen, q_light_gen, q_intensity, &movements, &est_sig, max_advance, max_turn_angle, genetic_path,
                 num_bits_vq, intensity);

                    inner_state ==1;
                }
                else{
                   movements.twist=0.0;
                    movements.advance =0.0;
                    inner_state == 0; 
                }
            }
            //printf("K_INTENSITY: %f,  q_intensity: %d",K_INTENSITY,q_intensity);
            else
            flg_result = state_machine_engine_stochastic(q_inputs_gen, q_light_gen, q_intensity, &movements, &est_sig, max_advance, max_turn_angle, genetic_path,
                     num_bits_vq, intensity);

            if(flg_result == 1) flg_stop=1;
            break;

            //DERIVED HMM FSM
            case 13:

            if(flagOnce)
            {
               est_sig = 0; //flg old?
               flagOnce = 0;
            }
            if (params.flg_genetics){sprintf(file_hmm,"data/avoid_hmm_%i.dat",params.individuo);}
            if(params.useRealRobot){
                if(inner_state == 0){
                    flg_result = state_machine_destination_hmm(q_light_gen, q_intensity, q_inputs_gen, &est_sig,
                                max_advance, max_turn_angle,  file_hmm, genetic_path, num_bits_vq, &movements, intensity);

                    inner_state ==1;
                }
                else{
                   movements.twist=0.0;
                    movements.advance =0.0;
                    inner_state == 0; 
                }
            }
            //printf("K_INTENSITY: %f,  q_intensity: %d",K_INTENSITY,q_intensity);
            else
            flg_result = state_machine_destination_hmm(q_light_gen, q_intensity, q_inputs_gen, &est_sig,
                                max_advance, max_turn_angle,  file_hmm, genetic_path, num_bits_vq, &movements, intensity);
            if(flg_result == 1) flg_stop=1;
            break;


            //DERIVED MDP 
            case 14: 
             if(flagOnce)
            {
               est_sig = 0;
               flagOnce = 0;
               inner_state = 0;//real robot
               num_obs = 1;
               rotate=1; //rotate to light source
            }
            printf("%s",params.file_behavior);
            printf("q_destination %i \n",q_light_gen);
            angle_mdp = inverse_quantize_destination(q_light_gen);
            if (params.flg_genetics){sprintf(object_file,"avoid_mdp_%i.dat",params.individuo);}
            else{sprintf(object_file,"mdp_environment.mdp");}


            //if(inner_state==0 && (num_obs%NUM_POLICY == 0 || num_obs==1) && rotate==1){
            if(inner_state==0 &&  num_obs==1 && rotate==1){
                printf("Rotating to light source \n");
                // It robot first rotates to the light source       
                movements.twist=angle_mdp;
                movements.advance =0.0;
                //inner_state = 1;
                flg_result = 0;
                rotate = 0;
            }
            else if(params.useRealRobot){
                printf("inside real robot");
                if(inner_state == 1){
                    movements.twist=0.0;
                    movements.advance =0.0;
                    inner_state == 2;
                }
                else{
                    flg_result = go_to_mdp_gen_stalin(num_obs,q_light_gen,q_intensity,lidar_readings, &movements, genetic_path,params.laser_value, params.laser_num_sensors, max_advance, max_turn_angle,intensity, object_file);
                    //inner_state = 1;
                    inner_state=0;
                    num_obs += 1;
                    rotate=1;
                }
            }
            else{
                printf("inside simultor goto mdp, num_obs = %i",num_obs);
                flg_result =go_to_mdp_gen_stalin(num_obs,q_light_gen,q_intensity,lidar_readings, &movements, genetic_path,params.laser_value, params.laser_num_sensors, max_advance, max_turn_angle,intensity, object_file);
                angle_mdp = inverse_quantize_destination(q_light_gen);
                movements.twist=-movements.twist+angle_mdp-0.785398/2;    
                num_obs+=1;
                rotate=1;
            }


            if(flg_result == 1) flg_stop=1;
            break;

            //For testing
            case 15:
            if(flagOnce)
            {
               est_sig = 0;
               flagOnce = 0;
            }
            	//LASER TEST
                /*movements.twist=0.0;
                movements.advance=0.0;
                fprintf(test, "[");
                for(i=0;i<params.laser_num_sensors;i++)
                {

                	fprintf(test, "%f,",lidar_readings[i]);
                }
                fprintf(test, "]\n");
                if(cta_steps>=1000) flg_stop=1;
                */
                
            	//MOTOR TEST
            /*if(est_sig == 0){//un pasito pa delante
            	front=lidar_readings[3];
            	movements.twist = 0.00;
            	movements.advance = max_advance;
            	est_sig = 1;
            }
            else if (est_sig == 1) {//stop
            	movements.twist = 0.0;
            	movements.advance = 0.0;
            	est_sig = 2;
            }
            else if (est_sig == 2) {//un pasito pa tras
            	movements.twist = 0.0;
            	movements.advance = -1.2*max_advance;
            	y = front - 1.2*max_advance;
            	y_hat = lidar_readings[3];
            	fprintf(test, "[%f,%f,",y,y_hat);
            	back=lidar_readings[7];
            	est_sig = 3;
            }
            else if (est_sig == 3) {//stop
            	movements.twist = 0.0;
            	movements.advance = 0.0;
            	est_sig = 4;
            }
            else if (est_sig == 4){ //pa delante ciclo
            	movements.twist = 0.00;
            	movements.advance = max_advance;
            	y = back - max_advance;
            	y_hat = lidar_readings[7];
            	fprintf(test, "%f,%f]\n",y,y_hat);
            	front=lidar_readings[3];
            	est_sig = 1;
            }
            else {
            	printf("estado NA");
            }
            if(cta_steps>=11){
            	movements.twist = 0.0;
            	movements.advance=0.0;
            	flg_stop=1;
            }
            */
			

            //SINGLE STEP TEST
            movements.advance = max_advance;
            movements.twist = max_turn_angle;
            if(cta_steps>=1){
                movements.twist = 0.0;
                movements.advance=0.0;
                //flg_stop=1;
            }

            //TURN TEST
            /*if(est_sig == 0){//un pasito pa delante
            	movements.twist = max_turn_angle;
            	movements.advance = 0.0;
            	est_sig = 1;
            }
            else if (est_sig == 1) {//stop
            	movements.twist = 0.0;
            	movements.advance = 0.0;
            	est_sig = 2;
            }
            else if (est_sig == 2) {//un pasito pa tras
            	movements.twist = max_turn_angle;
            	movements.advance = 0.0;
            	est_sig = 3;
            }
            else if (est_sig == 3) {//stop
            	movements.twist = 0.0;
            	movements.advance = 0.0;
            	est_sig = 4;
            }
            else if (est_sig == 4){ //pa delante ciclo
            	movements.twist = max_turn_angle;
            	movements.advance = 0.0;
            	est_sig = 1;
            }
            else {
            	printf("estado NA");
            }
            if(cta_steps>=21){
            	movements.twist = 0.0;
            	movements.advance=0.0;
            	flg_stop=1;
            }
            */
                break;
             default:
                    printf(" ******* SELECTION NO DEFINED *******\n");
                    movements.twist = 3.1416/4;
                    movements.advance = .03;
                break;
            }
            if(cta_steps == params.steps-2){
                flg_stop=1;
                printf("stop \n");
            }
            if(flagRun==1 && flg_stop ==1){
                fprintf(fpw,"( distance %f )\n",sqrt((params.robot_x-params.light_x)*(params.robot_x-params.light_x)+(params.robot_y-params.light_y)*(params.robot_y-params.light_y)));
                fprintf(fpw,"( num_steps %d )\n",cta_steps);
                fclose(fpw);
                fpw = NULL;
                fclose(test);
                stop();
                flagRun=0;
                flg_stop=0;
                printf("filesClosed");
                NUM_RUN+=1;
                printf("RUN: %i",NUM_RUN);
            }
            ros::spinOnce();
            // printf("\n\n             MOTION PLANNER \n________________________________\n");
            // printf("Light: x = %f  y = %f \n",params.light_x,params.light_y);
            // printf("Robot: x = %f  y = %f \n",params.robot_x,params.robot_y);
            // printf("Step: %d of %i \n",cta_steps,params.steps);
            // printf("Movement: twist: %f advance: %f \n" ,movements.twist ,movements.advance );
            cta_steps++;
            flg_noise = params.noise;
            if(params.useRealRobot){
                if (movements.twist < 0) movements.twist = movements.twist + 0.096;
                else if (movements.twist > 0) movements.twist = movements.twist - 0.096;
                if (movements.advance < 0) movements.twist = movements.twist + 0.055;
                else if (movements.advance > 0) movements.twist = movements.twist - 0.055;
            }
            move_robot(movements.twist,movements.advance,lidar_readings);
            // it saves the robot's actions
            if(fpw!=NULL)
                fprintf(fpw,"( movement %f %f )\n",movements.twist,movements.advance);
            ros::spinOnce();
            new_simulation = 0;
            r.sleep();
            r.sleep();
            r.sleep();
            r.sleep();
            r.sleep();
            
        }
        ros::spinOnce();
        
        r.sleep();
    }
    fclose(fpw);
}
