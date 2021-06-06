/********************************************************************************
*       vq.h	                                                                *
*                                                                               *
*                               J. Savage                                       *
*                                                                               *
*                               FI-UNAM 2019                                    *
********************************************************************************/


// it reads the centroids
int read_centroids(char *str1, Centroid *centroids, int size_vectors, int size_quantizer,char *path){
 int num,num_vectors,j;
 char file_centroids[300];
 char dummy1[100];
 FILE *fpr;
 int flag;
 float s1;

 sprintf(file_centroids,"%s%s_%d.dat",path,str1,size_quantizer);

#ifdef DEBUG
 printf("\n *** Centroid %s\n",file_centroids);
#endif


 if((fpr=fopen(file_centroids,"r")) == NULL){
        printf("File %s can not be open\n",file_centroids);
        exit(0);
 }

 num_vectors=1;
 while(1){

                  if(fscanf(fpr,"%s%d",dummy1,&num)==EOF){
                        fclose(fpr);
                        return(num_vectors-1);
                  }

                  for(j=1;j<=size_vectors;j++){
                                fscanf(fpr,"%f",&s1);
                                centroids[num_vectors].sensors[j]=s1;
#ifdef DEBUG
                                printf("centroids[%d].sensors[%d] %f\n",num_vectors,j,centroids[num_vectors].sensors[j]);
#endif
                  }

                  flag=1;
                  while(flag==1){
                        fscanf(fpr,"%s",dummy1);
                        if(strcmp(")",dummy1) == 0)flag=0;
                  }

                  centroids[num_vectors].num=num;

                  //for(j=1;j<=size_vectors;j++)
                                //printf("centroids[%d].sensors[%d] %f\n",num_vectors,j,centroids[num_vectors].sensors[j]);

                  num_vectors++;

 }


}




float get_distance(Centroid c,float *observations, int num, int ofst){

 int i;
 float distance;

 distance=0;

 for(i=1;i<=num;i++){
        // it calculates the distance for each of the sonars
        distance+=(c.sensors[i]-observations[i+ofst])*(c.sensors[i]-observations[i+ofst]);
	//printf("c.sensors[%d] %f v.sensors[%d] %f\n",i,c.sensors[i],i+ofst,v.sensors[i+ofst]);
 }
 distance=(float) sqrt(distance);
 //printf("Distance: %f\n",distance);
 return(distance);
}


// it gets the closest centroid for a raw data
int get_closest_centroid(Centroid *centroids,float *observations,int num_centroids, int num, float *dst, int ofst){

 int i,id_cnt;
 float smallest_distance,distance;


 smallest_distance=get_distance(centroids[1],observations,num, ofst);
 id_cnt=1;
 //printf("distance from 1 centroid %f \n",smallest_distance);
 for(i=2;i <= num_centroids;i++){
         distance=get_distance(centroids[i],observations,num,ofst);
         //printf("distance from %d centroid %f \n",i,distance);
         if(distance<smallest_distance){
                 smallest_distance=distance;
                 id_cnt=i;
         }
 }

 *dst=smallest_distance;
 return(id_cnt-1);
}


