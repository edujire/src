
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


 mag= sqrt(1/intensity)-0.2;
 
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

float inverse_quantize_destination(int value){

 float angle;

 if(value == 0)      angle = PI / 4;
 else if(value == 1) angle = PI/2;
 else if(value == 2) angle =3*PI/4;
 else if(value == 3) angle = PI;
 else if(value == 4) angle = 5*PI/4;
 else if(value == 5) angle = 6*PI/4;
 else if(value == 6) angle = 7*PI/4;
 else if(value == 7) angle = 0;
 else angle = 0;

 return angle;

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
int quantize_inputs(float *observations, int size_vectors, int size_quantizer, char *path, float scale){

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
 float n_observations[100];

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
	n_observations[i]=observations[i]*scale;
        printf("observations[%d] %f\n",i,observations[i]);
	
   }


   // function in ../vq/vq.h
   value = get_closest_centroid(centroids,n_observations,num_centroids,size_vectors,&dst,ofst);
   //printf(" index %d distance %f\n",value,dst);
   printf("closest centroid: %d \n",value);
 

 return(value);

}

// it writes the sensors' readings
int write_obs_sensor(FILE *fpw,float *observations,int num_sensors, float start_angle,float range){
 int j;

//observations from raw to *float
 fprintf(fpw,"( sensor laser %d %f %f",num_sensors,range,start_angle); 

 /*for(j=0;j<num_sensors;j++){
        fprintf(fpw," %f",observations[j]);
        //printf("laser[%d] %f\n",j,observations.sensors[j]);
 }
*/
 fprintf(fpw," )\n");
 //printf("\n");

 return(1);
}

int get_index_range(float obs, float r1, float r2, float r3){

 int index = 0;

 if((obs < r3) & (obs >= r2)){
  index = 3;  
 }
 else if((obs < r2) & (obs >= r1)){
  index = 2;  
 }else if (obs < r1){
  index = 1;
 }

 return index;

}




//void get_source(int quantized_attraction, float x, float y, int *j, int *k){
void get_source(int quantized_attraction, int *j, int *k){

// Check how the quantized attraction value is generated in function quantize_destination in ..//utilities/utilities.h
#ifdef DEBUG
 printf("quantized destination %d \n",quantized_attraction);
#endif
 // j represents the raw and k the columns
// if((x > 0.0) && (y > 0.0)){
 if(quantized_attraction == 0){
    // E
          *j=3;
          *k=6;
#ifdef DEBUG
    printf("Light source in the E\n");
#endif
          //coor_destination.xc= 0.0001;
          //coor_destination.yc= 0.0001;
 }
 else if(quantized_attraction == 1){
          // NE
          *j=6;
          *k=6;
#ifdef DEBUG
    printf("Light source in the NE\n");
#endif
          //coor_destination.xc= 1.0001;
          //coor_destination.yc= 1.0001;
 }
 else if(quantized_attraction == 2){
          // N
          *j=6;
          *k=3;
#ifdef DEBUG
    printf("Light source in the N\n");
#endif
          //coor_destination.xc= 0.0001;
          //coor_destination.yc= 1.0001;
 }
 else if(quantized_attraction == 3){
          // NW
          *j=6;
          *k=1;
#ifdef DEBUG
    printf("Light source in the NW\n");
#endif
          //coor_destination.xc= -1.0001;
          //coor_destination.yc= -1.0001;
 }
 else if(quantized_attraction == 4){
          // W
          *j=3;
          *k=1;
#ifdef DEBUG
    printf("Light source in the W\n");
#endif
          //coor_destination.xc= -1.0001;
          //coor_destination.yc= 0.0001;
 }
 else if(quantized_attraction == 5){
          // SW
          *j=1;
          *k=1;
#ifdef DEBUG
    printf("Light source in the SW\n");
#endif
          //coor_destination.xc= -1.0001;
          //coor_destination.yc= -1.0001;
}
else if(quantized_attraction == 6){
          // S
          *j=1;
          *k=3;
#ifdef DEBUG
    printf("Light source in the S\n");
#endif
          //coor_destination.xc= 0.0001;
          //coor_destination.yc= -1.0001;
}
else if(quantized_attraction == 7){
          // SE
          *j=1;
          *k=6;
#ifdef DEBUG
    printf("Light source in the SE\n");
#endif
          //coor_destination.xc= 1.0001;
          //coor_destination.yc= -1.0001;
}
else{
    *j=6;
          *k=6;
#ifdef DEBUG
          printf("Light source not found\n");
#endif
}

}


void ocupancy_grid_mdp(float *observations, int size, int attraction, struct mdp_database *mdp, float range_sensor){

  int i,j,jj,k;
  float obs,r1,r2,r3;
  int index;
  float x,y;


  // It initializes the MDP occupancy grid
  for(j=1,jj=mdp->num_rows;j<=mdp->num_rows;j++,jj--){
        for(k=1;k<=mdp->num_columns;k++){
                mdp->type[jj][k]=1;
                //printf("type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
                mdp->reward[jj][k]=-0.040000;
                //printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
        }
  }

#ifdef DEBUG
  for(i=0; i < size;i++){
        printf("%d %f\n",i,observations[i]); 
  }
#endif

  r1 = range_sensor/3.0;
  r2 = range_sensor/2.0;
  r3 = range_sensor;

#ifdef DEBUG
  printf("range sensor %f\n",range_sensor);
#endif

  obs= observations[0];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[0] %f\n",observations[0]);
#endif
        jj = 3;
        k = 3 + index;
        mdp->type[jj][k]=0;
#ifdef DEBUG
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        mdp->reward[jj][k]=-1.00000;
#ifdef DEBUG
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }

  obs= observations[1];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[1] %f\n",observations.sensors[1]);
#endif
        jj = 3 + index-1;
        k = 3 + index-1;
        mdp->type[jj][k]=0;
#ifdef DEBUG
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        mdp->reward[jj][k]=-1.00000;
#ifdef DEBUG
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }


  obs= observations[2];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[2] %f\n",observations.sensors[2]); 
#endif
        jj = 3 + index-1;
        k = 3;
        mdp->type[jj][k]=0;
#ifdef DEBUG
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        mdp->reward[jj][k]=-1.00000;
#ifdef DEBUG
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }

  obs= observations[3];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[3] %f\n",observations.sensors[3]); 
#endif
        jj = 3 - index+1;
        k = 3;
        mdp->type[jj][k]=0;
#ifdef DEBUG
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        mdp->reward[jj][k]=-1.00000;
#ifdef DEBUG
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }

  obs= observations[4];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[4] %f\n",observations.sensors[4]); 
#endif
        jj = 3 ;
        k = 3 - index+1;
        mdp->type[jj][k]=0;
#ifdef DEBUG
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        mdp->reward[jj][k]=-1.00000;
#ifdef DEBUG
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }

  obs= observations[5];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[5] %f\n",observations.sensors[5]); 
#endif
        jj = 3 - index +1 ;
        k = 3 - index + 1;
        mdp->type[jj][k]=0;
#ifdef DEBUG
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        mdp->reward[jj][k]=-1.00000;
#ifdef DEBUG
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }
 
  obs= observations[6];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[6] %f\n",observations.sensors[6]); 
#endif
        jj = 3 - index +1;
        k = 3 ;
        mdp->type[jj][k]=0;
#ifdef DEBUG
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        mdp->reward[jj][k]=-1.00000;
#ifdef DEBUG
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }

  obs= observations[7];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[7] %f\n",observations.sensors[7]); 
#endif
        jj = 3 - index +1;
        k = 3 + index -1;
        mdp->type[jj][k]=0;
#ifdef DEBUG
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        mdp->reward[jj][k]=-1.00000;
#ifdef DEBUG
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }



#ifdef DEBUG
 printf("Quadrant attractor %d\n",attraction);
#endif

 get_source(attraction,&j,&k);

 mdp->type[j][k]=2;
#ifdef DEBUG
 printf("type[%d][%d] %d\n",j,k,mdp->type[j][k]);
#endif
 mdp->reward[j][k]=2.04;
#ifdef DEBUG
 printf("reward[%d][%d] %f\n",j,k,mdp->reward[j][k]);
#endif


}




/* Function that reads the mdp*/
void write_mdps(struct mdp_database mdp, char *path)
{
   FILE *fp2;
   int i,j,k,jj;
   char input_file[300];


   /* It opens the mdp file */
   sprintf(input_file,"%smdp_environment_grid.mdp",path);
#ifdef DEBUG
   printf("writting MDP FILE: %s\n",input_file);
#endif
   fp2=fopen(input_file,"w");
   if(fp2==0){
        printf("\n File %s can not be open",input_file);
        exit(0);
   }

   /* It writes the number of rows and columns */
   fprintf(fp2,"%d %d\n",mdp.num_rows,mdp.num_columns);


   // It writes the type and the rewards of the MDP celds 
   for(j=1,jj=mdp.num_rows;j<=mdp.num_rows;j++,jj--){
        for(k=1;k<=mdp.num_columns;k++){
                fprintf(fp2,"%d ",mdp.type[jj][k]);
                //printf("type[%d][%d] %d\n",jj,k,mdp.type[jj][k]);
                fprintf(fp2,"%f ",mdp.reward[jj][k]);
                //printf("reward[%d][%d] %f\n",jj,k,mdp.reward[jj][k]);
        }
	fprintf(fp2,"\n");
   }


   // It prints the probabilities 
   for(i=1;i<=8;i++){ 
    for(j=1;j<=ROW_PRB;j++){
        for(k=1;k<=CLM_PRB;k++){
                fprintf(fp2,"%f ",mdp.pr[i][j][k]);
                //printf("pr[%d][%d][%d] %f\n",i,j,k,mdp.pr[i][j][k]);
        }
	fprintf(fp2,"\n");
    }
   }


   fclose(fp2); 

}

