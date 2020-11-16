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
    // it sets the environment's path
    strcpy(path,"./src/simulator/src/data/");

    FILE *test;
    char file_test[200];

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
                for( i = 0; i < 512; i++)
                    lidar_readings[i] = lasers[i];
            }

            // it quantizes the sensory data
            q_light = quantize_light(light_readings); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h
            
            if(params.noise )
                q_inputs = quantize_laser_noise(lidar_readings,params.laser_num_sensors,params.laser_value); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h
            else
                q_inputs = quantize_laser(lidar_readings,params.laser_num_sensors,params.laser_value); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h


            max_advance = params.robot_max_advance;
            max_turn_angle = params.robot_turn_angle;
            //genetic behavior quantized variables
            K_INTENSITY=2.52*max_advance;
            q_intensity = quantize_intensity(intensity);
            q_light_gen = quantize_destination(light_readings);
            q_inputs_gen = quantize_inputs(lidar_readings, params.laser_num_sensors, size_vq, genetic_path);
            //saves robot info
            if(flagOnce==1){
                sprintf(file_obs,"%sdata/output.raw",genetic_path);     //  cambiar por behavior_#ind.raw crear dentro del bucle con nombre distinto cada vez igual el fileclose
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


            switch ( params.behavior)
            {

            case 1:
                // This function sends light sensory data to a function that follows a light source and it issues
                // the actions that the robot needs to perfom.
                // It is located in ~/catkin_ws/src/simulator/src/state_machines/light_follower.h
                flg_result = light_follower(intensity, light_readings,&movements,max_advance,max_turn_angle);
                if(flg_result == 1) stop();
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
                if(flg_result == 1) stop();

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
                flg_result=sm_avoidance_destination(intensity,q_light,q_inputs,&movements,&est_sig,
                                                    params.robot_max_advance ,params.robot_turn_angle);

                if(flg_result == 1) stop();
                break;

            case 5:
                // This function sends the intensity and the quantized sensory data to a Clips node and it receives the actions that
                // the robot needs to perfom to avoid obstacles and reach a light source according to first order logic
                // It is located in ~/catkin_ws/src/simulator/src/behaviors/oracle.h
                result=oracle_clips(intensity,q_light,q_inputs,&movements,max_advance ,max_turn_angle);
                if(result == 1.0) stop();
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
                            stop();
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
                        if(flg_finish == 1) stop();
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
            flg_result = state_machine_engine(q_inputs_gen, q_light_gen, q_intensity, &movements, &est_sig, max_advance, max_turn_angle,
                genetic_path, num_bits_vq, intensity);

            if(flg_result == 1) stop();
            break;

            //DERIVED FSM STOCHASTIC
            case 12:

            if(flagOnce)
            {
               est_sig = 0;
               flagOnce = 0;
            }
            //printf("K_INTENSITY: %f,  q_intensity: %d",K_INTENSITY,q_intensity);
            flg_result = state_machine_engine_stochastic(q_inputs_gen, q_light_gen, q_intensity, &movements, &est_sig, max_advance, max_turn_angle, genetic_path,
                     num_bits_vq, intensity);

            if(flg_result == 1) stop();
            break;

            case 13:
                write_obs_sensor(test,lidar_readings,params.laser_num_sensors,params.laser_origin,params.laser_range);
                movements.twist=0.0;
                movements.advance=0.0;
                if(cta_steps>=10) stop();
                break;
             default:
                    printf(" ******* SELECTION NO DEFINED *******\n");
                    movements.twist = 3.1416/4;
                    movements.advance = .03;
                break;
            }

            ros::spinOnce();
            printf("\n\n             MOTION PLANNER \n________________________________\n");
            printf("Light: x = %f  y = %f \n",params.light_x,params.light_y);
            printf("Robot: x = %f  y = %f \n",params.robot_x,params.robot_y);
            printf("Step: %d \n",cta_steps++);
            printf("Movement: twist: %f advance: %f \n" ,movements.twist ,movements.advance );

            flg_noise = params.noise;

            move_robot(movements.twist,movements.advance,lidar_readings);
            // it saves the robot's actions
            fprintf(fpw,"( movement %f %f )\n",movements.twist,movements.advance);
            ros::spinOnce();
            new_simulation = 0;
            r.sleep();
        }
        ros::spinOnce();
        if(flagRun==1){
            fprintf(fpw,"( distance %f )\n",sqrt((params.robot_x-params.light_x)*(params.robot_x-params.light_x)+(params.robot_y-params.light_y)*(params.robot_y-params.light_y)));
            fprintf(fpw,"( num_steps %d )\n",cta_steps);
            fclose(fpw);
            flagRun=0;
        }
        r.sleep();
    }
    //fclose(fpw);
}
