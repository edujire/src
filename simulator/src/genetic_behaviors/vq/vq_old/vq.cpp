/********************************************************
*	vq.cpp					*
*							*
*							*
*		University of Mexico U.N.A.M.		*
*			Jesus Savage			*
*							*
*				11/15/2019		*
*				11/15/2019		*
*********************************************************
*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "allocate.h" 
#include "vq_lib.h" 


/* It creates the vector quantizer */
void get_vq(float **vectors, int num_vectors, int vectors_size, int vq_size, char * vq_file, float epsilon){


 float distante;
 float dtli,dt,dtl1,c99,c101,difference,prev_difference;
 float *vector,big,modifier[3],*aux,eps;
 double dmirea,dtl,num_spaces,nlmax,*aux1,power_2(),dist_vect;
 double **rn,dmin;
 float **vector_patterns;
 int kj,*point,num_iterations,nl,l,lmin,bigger,*numdist,i,j,k,kk;
 int flag_part,m,ini,ifin;
 char file_name[128];
 FILE *fp1,*fp2,*fp3;
 float *lpc,*reflex_coff;
 int result,*index_vectors;
 float alfa;
 float **dist;




 /* It allocates the vectors and matrix used */

 if( (rn = (double **) alloc_matrix_double(vq_size+1, vectors_size+1)) == 0){
         printf("\n Memory allocation error ");
         exit(0);
 }

 if( (vector_patterns = (float **)alloc_matrix
					 (vq_size+1,vectors_size+1)) == 0){
         printf("\n Memory allocation error ");
         exit(0);
 }

 if( (dist = (float **)alloc_matrix(vq_size+1,vq_size+1)) == 0){
         printf("\n Memory allocation error ");
         exit(0);
 }

 
 if( (aux = (float *) alloc_vector(vectors_size+1)) == 0){
         printf("\n Memory allocation error ");
         exit(0);
 }

 if( (vector = (float *) alloc_vector(vectors_size+1)) == 0){
         printf("\n Memory allocation error ");
         exit(0);
 }
  
 if( (aux1 = (double *) alloc_vector_double(vectors_size+1)) == 0){
         printf("\n Memory allocation error ");
         exit(0);
 }
  
 
 if( (point= (int *) alloc_vector_int(vq_size+1)) == 0){
         printf("\n Memory allocation error ");
         exit(0);
 }

 
 if( (numdist= (int *) alloc_vector_int(vq_size+1)) == 0){
         printf("\n Memory allocation error ");
         exit(0);
 }

 if( (index_vectors= (int *) alloc_vector_int(num_vectors+1)) == 0){
         printf("\n Memory allocation error ");
         exit(0);
 }




 /* It opens the vq file */
 fp1=fopen(vq_file,"w");
 if(fp1==0){
	printf("\n Error in opening %s",vq_file);
	exit(0);
 }

 /* It opens the index file */
 fp3=fopen("index.dat","w");
 if(fp3==0){
	printf("\n Error in opening index.dat");
	exit(0);
 }



 /* It initializes some variables */
 num_iterations=0;
 eps=epsilon;		
 difference=0;
 nlmax= vq_size;
 num_spaces= vq_size;
 ini=1;
 flag_part=SHOW_DISTORTION;
 for(k=1;k<= vq_size;k++){
	point[k]=0;
 	numdist[k]=0;
 }
 for(i=1;i<=vectors_size;i++){
	aux[i]=0.0;
	aux1[i]=0.0;
 }

 modifier[1]=.999;
 modifier[2]=1.001;
 c99=.999;
 c101=1.001;
 dtli=0.0;


 /* It finds the Main Ceptral Centroide */

 nl=2;
 kk=1;

 ifin=num_vectors;
 for(j=1;j<=ifin;j++){
	/*printf("\n Vector %d",j);*/
	for(i=1;i<=vectors_size;i++){
	  aux1[i]=aux1[i]+ vectors[j][i];
	  /*printf("\n vect[%d][%d] %f aux1 %f",j,i,vectors[j][i],aux1[i]);*/

 	}
 }
 
 printf("\n Number of vectors %d ",ifin-1);

 for(i=1;i<=vectors_size;i++){
		aux[i]=aux1[i]/ifin;
		/*printf("\n aux1[%d] %f average %f",i,aux1[i],aux[i]);*/
 }


 /* Modify the Space */
 for(j=1;j<3;j++){
	/* It modifies the centriod */
	printf("\n centroid[%d]",j);
        for(i=1;i<=vectors_size;i++){
             vector_patterns[j][i]=aux[i]*modifier[j];;
	     printf(" %f",vector_patterns[j][i]);
        }
 }


/* Separate the spaces into new partitions */
 while(nl<=nlmax){
	  dt=0.0;
	  dtl=0.0;

	  for(k=1;k<=nl;k++){
		point[k]=0;
		numdist[k]=0;
 		for(i=1;i<=vectors_size;i++)rn[k][i]=0.0;
	  }

	  for(l=1;l<ifin;l++){
		/* It selects one vector */
 		for(i=1;i<=vectors_size;i++){
		  vector[i]=vectors[l][i];
          	  /*printf("\n vectors[%d] %f",i,vector[i]);*/
 		}

	       /* Find the vector_pattern that is closest to the vector */
	       find_closest_pattern(vector,vector_patterns,vectors_size,nl,
								&dmin,&lmin);

		index_vectors[l]=lmin;
		numdist[lmin]++;
		point[lmin]++;
		dt=dt+dmin;
		dtl=dtl+(float)log(dmin+1.);
		if(flag_part)
			printf("\n %d index %d distortion %f total_dist %f",
				l,lmin,dmin,dtl);

		/* It adds the vectors in one region together */
		for(i=1;i<=vectors_size;i++)rn[lmin][i]=rn[lmin][i]+vector[i];

	  }

	  prev_difference=difference;
	  difference = abs1(dtl-dtli);

#ifdef VQ_DEBUG 
	  /*printf("\n\n Epsilon %f Iteration %d",eps,num_iterations);*/
	  printf("\n\n Iteration %d",num_iterations);
	  for(l=1;l<nl+1;l++){
		printf("\n Space %d num.vect %d",l,point[l]);
	  }
	  printf("\n dist. %e previous dist. %e",dtl,dtli);
	  printf("\n difference %e prev_difference %e\n",
			difference,prev_difference);

#endif
	  /* It checks if there were partitions without vectors */
	  for(k=1;k<nl+1;k++){
	    if(numdist[k]==0){
#ifdef VQ_DEBUG 
		printf("\n '%d'is a partition without vectors",k);
#endif

	  	/*It finds the partion with more vectors to split it
		  into two parts */
		bigger=point[1];
		kj=1;

		for(i=2;i<nl+1;i++){
			if(point[i]> bigger){
				kj=i;
				bigger=point[i];
			}
		}

		for(i=1;i<=vectors_size;i++){
			rn[k][i]=(rn[kj][i]*c99)/(numdist[kj]);
			rn[kj][i]=(rn[kj][i]*c101);
		  	/*printf("\n rn[%d][%d] %f", k,i,rn[k][i]);
		  	printf("\n rn[%d][%d] %f", kj,i,rn[kj][i]);*/
		}

		numdist[k]=1;
		point[k]=1;
	     }

	  }



	/* It checks if we finally found the optimum space separation */
	if( ( abs1(dtl-dtli) >= eps) &
		(abs1(difference-prev_difference) >= eps)){

  	    for(k=1;k<nl+1;k++){
		/* It updates the centroids */
		printf("\n centroid[%d]",k);
		for(i=1;i<=vectors_size;i++){
			vector_patterns[k][i]=(float)rn[k][i]/numdist[k];
			printf(" %f",vector_patterns[k][i]);
                }

	     }

	    num_iterations++;
	    if(num_iterations > NUM_MAX_ITERATIONS){
		eps=eps+epsilon;		
		num_iterations=0;
	    }
	    dtli=dtl;	/* dtl = total distortion */
        }


	/* An optimum partition was found */
	else{
           num_iterations=1;
	   kk=1;

	   for(k=1;k < nl+1;k++){

                /* It updates the centroids */
                for(i=1;i<=vectors_size;i++){
                        vector[i]=(float)rn[k][i]/numdist[k];

                }

	   	/* Save the patterns or modify the space */
		if(nl==vq_size){
		  fprintf(fp1,"(centroid %d",k); 	
		  for(i=1;i<=vectors_size;i++){
		       /*printf("\n %s vq[%d][%d]= %f",vq_file,k,i,vector[i]);*/
			fprintf(fp1," %f",vector[i]);
		  }
		  fprintf(fp1,")\n");
		  /*printf("\n");*/

		 }

		 else{
			/* Modify Again the Space */
 			for(j=1;j<3;j++){
                		/* It modifies the centroid */
				if(flag_part)
				printf("\n centroid[%d]",kk);
                		for(i=1;i<=vectors_size;i++){
                        	      vector_patterns[kk][i]=aux[i]*modifier[j];
					if(flag_part)
                        		printf(" %f",vector_patterns[kk][i]);
                		}
				kk++;

 			}

		  }

	   }

	   nl=nl+nl;
	   dtli=0.0;
 	   c99=modifier[1];
 	   c101=modifier[2];
 	   /*eps=epsilon + .001;*/

          }


 }
 fclose(fp1);


 /* It saves each vector and its centroid */
 for(l=1;l<ifin;l++){
      fprintf(fp3,"(vector %d %d",l,index_vectors[l]);
      for(i=1;i<=vectors_size;i++){
          fprintf(fp3," %f",vectors[l][i]);
      }
      fprintf(fp3,")\n");
 }


#ifdef VQ_SPECS 

 big=0;

 /* It opens the space file */
 strcpy(file_name,vq_file);
 strcat(file_name,"_spa");
 fp2=fopen(file_name,"w");

 /* It saves the number of vectors in each partion */
 for(i=1;i<=num_spaces;i++){
	/*printf("\n Space %d num.vect %d",i,point[i]);*/
	fprintf(fp2," Space %d num.vect %d\n",i,point[i]);

	for(k=1;k<=vectors_size;k++){
	  vector[k]=(float)rn[i][k];
 	}

	for(j=1;j<=num_spaces;j++){

		distance_vector(vectors_size,vector,vector_patterns[j],
							&dist_vect);
		dist[i][j]=(float)dist_vect;
		if(dist[i][j] > big)big=dist[i][j];
	}
 }
 
 /*printf("\n");*/
 fclose(fp2);

 /* It opens the distance file */
 strcpy(file_name,vq_file);
 strcat(file_name,"_dis");
 fp1=fopen(file_name,"w");


 for(i=1;i<=num_spaces;i++){
	for(j=1;j<=num_spaces;j++){
		dist[i][j]=dist[i][j]/big;
		/*printf("\n %s dist[%d][%d] %f",vq_file,i,j,dist[i][j]);*/
		fprintf(fp1,"%f\n",dist[i][j]);
	}
 }

 fclose(fp1);

 /* It transforms the distance matrix into a simetric matrix */
 /* It opens the simetric file */
 strcpy(file_name,vq_file);
 strcat(file_name,"_sim");
 fp1=fopen(file_name,"w");


 /* printf("\n Simetric Matrix");*/
 for(i=1;i<=num_spaces;i++){

		  for(j=1;j<=i;j++){

			 distante=(dist[j][i]+dist[i][j])/2;
			 dist[j][i]=distante;
	 		 dist[i][j]=distante;
		  	 /*printf("\n %s dist[%d][%d] %f dist[%d][%d] %f"
			,vq_file,i,j,dist[i][j],j,i,dist[j][i]);*/
		  }
 }

 for(i=1;i<=num_spaces;i++){
		  for(j=1;j<=num_spaces;j++){
			 fprintf(fp1,"%f\n",dist[i][j]);
		  }
 }
 /*printf("\n");*/
 fclose(fp1);

#endif

 /* It deallocates the vectors */
 free_matrix_double(rn,vq_size+1);
 free_matrix(vector_patterns,vq_size+1);
 /*free_matrix(dist,vq_size+1);*/
 /*free_vector(aux);*/
/*
 free_vector(vector);
 free_vector_double(aux1);
 free_vector_int(point);
 free_vector_int(numdist);
 free_vector(lpc);
 free_vector(reflex_coff);
*/

}




main( int   argc, char *argv[]) {

 int num_vectors,order_vectors,num_regions;
 char data_file[128],vq_file[128];
 float **vectors,epsilon;


 /* it gets line inputs */
 get_inputs(argc, argv,&num_regions,data_file,vq_file,&epsilon);


 /* it allocates memory space */
 num_vectors=get_size(data_file);
 order_vectors=2;
 if((vectors =(float **)alloc_matrix(num_vectors+1,order_vectors+1)) == 0){
         printf("\n Memory allocation error ");
         exit(0);
 }

 /* it reads the points */
 get_data(data_file,vectors);

 order_vectors=2;

 /* It creates the vector quantizer */
 get_vq(vectors,num_vectors,order_vectors,num_regions,vq_file,epsilon);

}

