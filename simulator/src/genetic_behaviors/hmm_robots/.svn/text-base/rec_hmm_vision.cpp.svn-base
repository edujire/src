//
// rec_hmm_vision.cpp
//
// This program recognizes an image pattern using HMM
// represent images' readings
//
// make -f Makefile_rec_hmm_vision
//
//  J. Savage, 4-22-2009
//      FI- UNAM
//


#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>




#include "/home/savage/vision/pattern_recognition/include_vision/basics_vqs_vision.h"
#include "/home/savage/vision/pattern_recognition/vq_vision/new_allocate.h"
#include "hmm_vision_lib.h"
#include "/home/savage/opencv/opencv_images.h"

const float fit_factor = 100.;

/* it gets line inputs */

int get_inputs(int argc, char **argv, char *objects_file, char *in_file, int *size_vq, char *path, char *vq_file, int *num_states, int *camera, int *scale) {
	int i;

	/* It sets default values */
	strcpy(objects_file, "vision_objects");
	strcpy(in_file, "test_image");
	*size_vq = 64;
	strcpy(path, "/home/savage/observations/cmucam/training_hsi/");
	strcpy(vq_file, "all");
	*num_states = NUM_STATES;
	*camera = -1;
	*scale = 1;


	/* reads input parameters */
	if (argc > 1) {
		for (i = 1;i < argc;i++) {
			if (!strcmp(argv[i], "-h")) {
				printf("rec_hmm_vision -h\n");
				printf("-f specifies the objects' file\n");
				printf("-s specifies the image's file\n");
				printf("-q specifies the VQ's file\n");
				printf("-m specifies the size of the quantizer\n");
				printf("-n specifies the number of states of the HMM\n");
				printf("-p specifies the path\n");
				printf("-c specifies camera number\n");
				printf("-r specifies reduction scale\n");
				printf("-h shows this message\n");
				printf("example:\n rec_hmm_vision -f vision_objects -s images_test_top_5 -m 64 -n 9 -p /home/savage/observations/cmucam/training_hsi/ -c 0 -r 2\n");

				exit(1);
			}
			if (!strcmp(argv[i], "-p")) {
				strcpy(path, argv[i+1]);
			}
			if (!strcmp(argv[i], "-s")) {
				strcpy(in_file, argv[i+1]);
			}
			if (!strcmp(argv[i], "-f")) {
				strcpy(objects_file, argv[i+1]);
			}
			if (!strcmp(argv[i], "-q")) {
				strcpy(vq_file, argv[i+1]);
			}
			if (!strcmp(argv[i], "-m")) {
				*size_vq = atoi(argv[i+1]);
			}
			if (!strcmp(argv[i], "-c")) {
				*camera = atoi(argv[i+1]);
			}
			if (!strcmp(argv[i], "-r")) {
				*scale = atoi(argv[i+1]);
			}
			if (!strcmp(argv[i], "-n")) {
				*num_states = atoi(argv[i+1]);
			}
		}
	} else {
		printf("Using default values \n");
	}

	printf("recognize_vision -f %s -s %s -m %d -p %s -q %s -n %d -c %d -r %d\n", objects_file, in_file, *size_vq, path, vq_file, *num_states, *camera, *scale);

	return(0);

}

struct vector_quantizer {
	char  in_file[300];
	int num_vectors;
	Centroid *centroids;
	Centroid *centroids_vision;
	int size_vectors; //=3
	Raw *vectors;
	char path[300];
	char data_file[300];
	int size_vq;
	int num_centroids_vision;
	char vq_file[300];
};

struct hmm_database {
	char objects_file[300], **objects_name;
	double bigger, prob[NUM_MAX_CENTROIDS], p;
	double vit_prob[NUM_MAX_CENTROIDS];
	int index;
	int index_vit ;
	double bigger_vit;
	int num_objects;
	float ***ais, **pis, ***bis;
	int num_states;
	int  *seq_states[10];
	int *observations;
};

void vq_initialize(struct vector_quantizer *vq) {
	if ((vq->centroids_vision = (Centroid *)
	        malloc( (vq->size_vq + 1) * sizeof(Centroid))) == NULL) {
		fprintf(stdout, "Allocation error: centroids");
		exit(0);
	}
	vq->size_vectors = 3;
	vq->num_centroids_vision = read_vision_centroids(vq->vq_file, vq->centroids_vision, vq->size_vectors, vq->size_vq, vq->path);
}

void vq_allocate_vectors(struct vector_quantizer *vq) {
	if ((vq->vectors = (Raw *) malloc( (vq->num_vectors + 1) * sizeof(Raw))) == NULL) {
		fprintf(stdout, "Allocation error: Raw vectors");
		exit(0);
	}
}

void hmm_initialize(struct hmm_database *hmm, struct vector_quantizer *vq) {
	int i,j;
	
	hmm->index = 1;
	hmm->index_vit = -1;

// it reads the names of the objects to be recognized
	if ( (hmm->objects_name = (char **) alloc_matrix_char(30, 300)) == 0) {
		printf("\n Memory allocation error ");
		exit(0);
	}

	hmm->num_objects = read_objects_name(hmm->objects_file, hmm->objects_name, vq->path);

	for (i = 1;i <= hmm->num_objects;i++) {
		printf("objects %s\n", hmm->objects_name[i]);
	}

//it reads each the objects' HMMs
// It allocates the matrices used for the hmm evaluation

	hmm->ais = (float ***)  malloc( (unsigned) (hmm->num_objects + 1) * sizeof(float *));
	hmm->bis = (float ***) malloc( (unsigned) (hmm->num_objects + 1) * sizeof(float *));
	hmm->pis = (float **)   malloc( (unsigned) (hmm->num_objects + 1) * sizeof(float *));

	for (j = 1;j <= hmm->num_objects;j++) {
		if ((hmm->ais[j] = (float **)alloc_matrix(hmm->num_states + 1, hmm->num_states + 1 )) == 0) {
			printf("\n Memory allocation error ");
			exit(1);
		}
		if ((hmm->bis[j] = (float **)alloc_matrix(hmm->num_states + 1, vq->size_vq + 1)) == 0) {
			printf("\n Memory allocation error ");
			exit(1);
		}
		if ((hmm->pis[j] = (float *)alloc_vector(hmm->num_states + 1)) == 0) {
			printf("\n Memory allocation error ");
			exit(1);
		}
	}

	if ((hmm->observations = (int *) alloc_vector_int((vq->num_vectors + 1)) ) == 0) {
		printf("\n Memory allocation error: observations ");
		exit(0);
	}

// it allocates the states vectors
	for (i = 1;i <= hmm->num_objects;i++) {
		if ((hmm->seq_states[i] = (int *) alloc_vector_int((vq->num_vectors + 1))) == 0) {
			printf("\n Memory allocation error: observations ");
			exit(0);
		}
	}

}

void q_free(struct vector_quantizer *vq) {
	free(vq->vectors);
	free(vq->centroids_vision);
	free(vq->centroids);
}

void hmm_free(struct hmm_database *hmm) {
	int i,j;
	
	for (i = 1;i <= hmm->num_objects;i++) {
		free(hmm->seq_states[i]);
	}

	for (j = 1;j <= hmm->num_objects;j++) {
		free(hmm->ais[j]);
		free(hmm->bis[j]);
		free(hmm->pis[j]);
	}
	free(hmm->ais);
	free(hmm->bis);
	free(hmm->pis);
	free(hmm->objects_name);
}

void hmm_recognize(struct hmm_database *hmm, struct vector_quantizer *vq) {
	int i, j, k;
	
	quantize_vectors(vq->centroids_vision, vq->num_vectors, vq->vectors, hmm->observations, vq->num_centroids_vision, vq->size_vectors);
	hmm->bigger = 0;
	hmm->bigger_vit = 0;


	for (i = 1;i <= hmm->num_objects;i++) {
		// It calculates the probabilities of each of the hmms
		hmm->prob[i] = get_hmm_prob(hmm->observations, hmm->ais[i], hmm->bis[i], hmm->pis[i], vq->num_vectors, hmm->num_states);
		// The result is divided by 10 in order that the result fits the variable size
		hmm->prob[i] = exp(hmm->prob[i] / fit_factor);
		//printf("object %s probability %e\n",objects_name[i],prob[i]);

		if (hmm->prob[i] > hmm->bigger) {
			hmm->bigger = hmm->prob[i];
			hmm->index = i;

		}

		/* It calculates the best sequence of states using the Viterbi algorithm
		 * and the probability of a hmm given a symbol vector */
		hmm->vit_prob[i] = viterbi(hmm->observations, hmm->ais[i], hmm->bis[i], hmm->pis[i], vq->num_vectors, hmm->num_states, hmm->seq_states[i]);
		hmm->vit_prob[i] = exp(hmm->vit_prob[i] / fit_factor);
		//printf("object %s Viterbi probability %e\n", hmm->objects_name[i], hmm->vit_prob[i]);


		if (hmm->vit_prob[i] > hmm->bigger_vit) {
			hmm->bigger_vit = hmm->vit_prob[i];
			hmm->index_vit = i;
		}
	}

	for (i = 1;i <= hmm->num_objects;i++) {
		printf("object %s probability %e\n", hmm->objects_name[i], hmm->prob[i]);
		printf("object %s Viterbi probability %e\n", hmm->objects_name[i], hmm->vit_prob[i]);
	}

	printf("\n\n ****** Winner object %s %e ***** \n", hmm->objects_name[hmm->index], hmm->prob[hmm->index]);
	printf("\n\n ****** Winner Vitterbi object %s %e ***** \n", (hmm->index_vit > -1) ? hmm->objects_name[hmm->index_vit] : "none", (hmm->index_vit > -1) ? hmm->vit_prob[hmm->index_vit] : 0.0000f);
}



int main(int argc, char *argv[]) {
	struct vector_quantizer vq;
	struct hmm_database hmm;
	float average = 0;
	CvCapture *cap = 0;
	int camera = -1;
	int scale = 1;
	

	/* it gets line inputs */
	get_inputs(argc, argv, hmm.objects_file, vq.in_file, &vq.size_vq, vq.path, vq.vq_file, &hmm.num_states, &camera, &scale);
	vq_initialize(&vq);

	//read image from camera and get size_vectors
	if (camera != -1) {
		set_capture_dimensions(640, 480);
		vq.num_vectors = get_max_vectors(camera, scale);
		cap = init_camera(camera);
	} else {
		sprintf(vq.in_file, "%s%s.raw", vq.path, vq.data_file);
		vq.num_vectors = get_size_image_vectors(vq.in_file, &vq.size_vectors);
	}
	printf("%s num. vectors %d size vectors %d\n", (camera == -1) ? vq.in_file : "Frame" , vq.num_vectors, vq.size_vectors);
	//Allocate VQ Vectors
	vq_allocate_vectors(&vq);
	//Initialize HMM database
	hmm_initialize(&hmm, &vq);
	/* It reads the hmm of each of the symbols */
	read_hmms(hmm.objects_name, hmm.num_objects, hmm.ais, hmm.bis, hmm.pis, hmm.num_states, vq.size_vq, vq.path);


	set_debug(1);
	
	while (1) {
		unsigned char c;
		IplImage *img = 0;

		if (camera != -1) {
			img = capture_picture_simple(cap);
			get_images_simple_img(img, scale, vq.vectors, RGB);
		} else {
			read_image_vectors(vq.in_file, vq.vectors);
		}
		
		hmm_recognize(&hmm, &vq);

		if (camera != -1) {
			c = cvWaitKey(1);

			if (c == 27) {
				close_camera(&cap);
				break;
			}
		}
		if (camera < 0) {
			break;
		}
		cvReleaseImage(&img);
	}
	


}



