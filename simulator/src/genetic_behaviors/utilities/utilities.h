
/********************************************************************************
*	utilities.h                                                           	*
* 									     	*
*                               J. Savage                                    	*
*                                                                        	*
*                               FI-UNAM 2015                                 	*
********************************************************************************/

extern float K_INTENSITY;

 // It quantizes the intensity 
int quantize_intensity(float intensity){

 int value=0;
 float mag;


 mag= sqrt(1/intensity);
 
if(mag > K_INTENSITY*4){
        value = 0;
}
else if(mag > K_INTENSITY*3){
        value = 1;
}
else if(mag > K_INTENSITY*2){
        value = 2;
}
else{
        value = 3;
}


 return value;

}


// It quantizes the destination this later 2-1-11
int quantize_destination(float *light_values){

int sensor = 0;

for(int i = 0; i < 8; i++ )
    {
        if( light_values[i] > light_values[sensor] )
            sensor = i;
    }
    //printf("sensor: %d \n",sensor);
    //printf("biggest value sensor %d %f\n",sensor,light_values[sensor]);
    if(sensor == 0){
        if(light_values[7]<light_values[1])
            return 0;
        else
            return 7;
    }
    else if(sensor == 1){
        if(light_values[0]<light_values[2])
            return 1;
        else
            return 0;
    }
    else if(sensor == 2){
        if(light_values[1]<light_values[3])
            return 2;
        else
            return 1;
    }
    else if(sensor == 3){
        if(light_values[2]<light_values[4])
            return 3;
        else
            return 2;
    }
    else if(sensor == 4){
        if(light_values[3]<light_values[5])
            return 4;
        else
            return 3;
    }
    else if(sensor == 5){
        if(light_values[4]<light_values[6])
            return 5;
        else
            return 4;
    }
    else if(sensor == 6){
        if(light_values[5]<light_values[7])
            return 6;
        else
            return 5;
    }
    else{
        if(light_values[6]<light_values[0])
            return 7;
        else
            return 6;
    }

}

// it generates a robot's output
movement generate_output_genetics(int out,float advance, float angle){

  movement output;
  switch(out){

        case 0: // Stop
                output.advance=0.0f;
                output.twist=0.0f;
#ifdef DEBUG
                printf("STOP\n");
#endif
                break;

        case 1: // Forward
                output.advance=advance;
                output.twist=0.0f;
#ifdef DEBUG
                printf("FORWARD\n");
#endif
                break;

        case 2: // backward
                output.advance=-advance;
                output.twist=0.0f;
#ifdef DEBUG
                printf("BACKWARD\n");
#endif
                break;

        case 3:// Turn left
                output.advance=0.0f;
                output.twist=angle;
#ifdef DEBUG
                printf("LEFT\n");
#endif
                break;

        case 4: // Turn right
                output.advance=0.0f;
                output.twist=-angle;
#ifdef DEBUG
                printf("RIGHT\n");
#endif
                break;

	case 5: // Turn right_advance
                output.advance=advance;
                output.twist=-angle;
#ifdef DEBUG
                printf("RIGHT_ADVANCE\n");
#endif
                break;

	case 6: // Turn left_advance
                output.advance=advance;
                output.twist=angle;
#ifdef DEBUG
                printf("LEFT_ADVANCE\n");
#endif
                break;

	case 7: // Turn right_advance twice
                output.advance=advance;;
                output.twist=-2*angle;
#ifdef DEBUG
                printf("RIGHT_ADVANCE TWICE\n");
#endif
                //output.advance=0.0f;
                //output.twist=0.0f;
                //printf("STOP\n");
                break;

        //case 8: // Turn left_advance twice
                //output.advance=advance;;
                //output.twist=2*angle;
                //printf("LEFT_ADVANCE TWICE\n");
                //break;


        default:printf("Output %d not defined used ", out);
                output.advance=0.0f;
                output.twist=0.0f;
                printf("STOP\n");
                break;
  }

 return output;
}


// It quantizes the inputs
int quantize_inputs(float *observations, int size_vectors, int size_quantizer, char *path){

 int value=0;
 int i;
 float left,right;
 int interval = size_vectors/2;
 static int num_centroids;
 static Centroid *centroids;
 static int flg_first = 1;
 char centroid_file[100];
 float dst; 
 int ofst=-1;
 float normalized_observations[40];

    if(flg_first==1){

    /* it allocates centroids space */
    if((centroids = (Centroid *)
                malloc( (NUM_MAX_CENTROIDS+1) * sizeof(Centroid))) == NULL){
                fprintf(stdout,"Allocation error: centroids");
                exit(0);
    }

    strcpy(centroid_file,CENTROID_FILE);
 
    /* it reads the centroids */
    // function in ../vq/vq.h
    num_centroids= read_centroids(centroid_file,centroids,size_vectors,size_quantizer,path);

    flg_first=0;
   }

   
   for(i=0;i<size_vectors;i++){
        printf("observations[%d] %f\n",i,observations[i]);
   }


   // function in ../vq/vq.h
   value = get_closest_centroid(centroids,observations,num_centroids,size_vectors,&dst,ofst);
   //printf(" index %d distance %f\n",value,dst);
   printf("closest centroid: %d \n",value);
 

 return(value);

}

// it writes the sensors' readings
int write_obs_sensor(FILE *fpw,float *observations,int num_sensors, float start_angle,float range){
 int j;

//observations from raw to *float
 fprintf(fpw,"( sensor laser %d %f %f",num_sensors,range,start_angle); 

 for(j=0;j<num_sensors;j++){
        fprintf(fpw," %f",observations[j]);
        //printf("laser[%d] %f\n",j,observations.sensors[j]);
 }

 fprintf(fpw," )\n");
 //printf("\n");

 return(1);
}
