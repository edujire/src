/*******************************************************************************
 *
 * Simple Random Boolean Network Complexity Metrics,  
 * Copyright (c) 2019
 * Centro de Ciencias de la Complejidad
 * http://c3.unam.mx
 * Universidad Nacional Autónoma de México.
 * All rights reserved.
 *  
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *  
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *  
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301  USA
 *******************************************************************************/
/**
 * @file mdpnav.c
 * @author Stalin Muñoz Gutiérrez
 * @date 3 March 2019
 * @brief Map Navigation with MDPs
 *
 * Basic Map Navigation with Markov Decision Processes
 * 
 */
#include "mdpnav.h"
#include "mdp.h"
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<unistd.h>
#include <float.h>

#define MDP_NUM_MAX_ITERATIONS 500


//char *ARROWS[8] = {"[ N ]","[ NE]","[ E ]","[ SE]","[ S ]","[ SW]","[ W ]","[ NW]"}; 
char *ARROWS[8] = {"N","A","E","I","S","O","W","U"};




void report_file(map *m,char *output_file){

    FILE *fp = (output_file!=NULL)?fopen(output_file, "w"):stdout;
   
#ifdef DEBUG 
    printf("\nResult in %s\n",output_file);
    printf("\nNotation North=N, North-East=A, East=E, South-East=I, South=S, South-West=O, West=W, North-West=U, Terminal=X\n");  
#endif

    for(int i=0;i<m->rows;i++){
        for(int j=0;j<m->columns;j++){
            if(m->cells[i][j].type==ACCESS){
                fprintf(fp,"%s",ARROWS[m->cells[i][j].pi]);
#ifdef DEBUG
                printf("%s ",ARROWS[m->cells[i][j].pi]);
#endif
            }
            else{
		if(m->cells[i][j].type==INACCESS){
                	fprintf(fp,"%s","Z");
#ifdef DEBUG
                	printf("%s ","Z");
#endif
                	//fprintf(fp,"%s",m->cells[i][j].type==INACCESS?"[▓▓▓]":"X");
                	//printf("%s ",m->cells[i][j].type==INACCESS?"[▓▓▓]":"X");
		}
		else{
			fprintf(fp,"%s","X");
#ifdef DEBUG
                        printf("%s ","X");
#endif

		}
            }
        }
        fprintf(fp,"\n");
#ifdef DEBUG
        printf("\n");
#endif
    }
    if(output_file!=NULL){
        fclose(fp);
    }
}

void parse_params(int argc, char *argv[], nav_spec *p){
    int opt;
    char *range = NULL;
    const char sep[2] = ":";
    while((opt = getopt(argc, argv, "foeg")) != -1){
        switch(opt){
            case 'f':
                p->input_file = argv[optind];
                break;
            case 'o':
                p->output_file = argv[optind]; 
                break;
            case 'g':
                p->gamma = atof(argv[optind]);
                break;
            case 'e':
                p->epsilon = atof(argv[optind]); 
                break;
        }
    }
}


LLST *empty(){
    LLST *list = (LLST *) malloc(sizeof(LLST));
    list->size = 0;
    list->first = NULL;
    list->last = NULL;
    return list;
}

void *head(LLST *list){
    if(list->size==0){
        return NULL;
    }
    link_ll *p = list->first;
    void *data = p->data;
    list->first = p->next;
    if(list->first!=NULL){
        list->first->previous = NULL;
    }
    list->size--;
    free(p);
    return data;
}


void free_LLST(LLST *list){
    while(list->size>0){
        head(list);
    }
    free(list);
}



void free_data_LLST(LLST *list){
    for(link_ll *p = list->first;p!=NULL;p = p->next){
        free(p->data);
    }
}




int best_expected_action_S(LLST **succs){
    int a, best_action;
    double u, best = -DBL_MAX;
    for(a=UP;a<=UP_LEFT;a++){
        u = expected_u_S(succs,a);
        if(u > best){
            best = u;
            best_action = a;
        }
    }
    return best_action;
}




double best_expected_u_S(LLST **succs){
    int a;
    double u, best = -DBL_MAX;
    for(a=UP;a<=UP_LEFT;a++){
        u = expected_u_S(succs,a);
        if(u > best){
            best = u;
        }
    }
    return best;
}

double expected_u_S(LLST **succs,int a){
    double u=0;
    SP *sp;
    for(link_ll *p=succs[a]->first;p!=NULL;p=p->next){
        sp = (SP *) p->data;
        u += sp->prob*sp->s->v;
    }
    return u;
}



void append(LLST *list,void *data){
    link_ll *l = new_link_ll(data,list->last,NULL);
    if(list->first==NULL){
        list->first=l;
        list->last=l;
    }else{
        list->last->next=l;
        list->last = l;
    }

    list->size++;
}


link_ll *new_link_ll(void *data,link_ll *previous,link_ll *next){
    link_ll *l = (link_ll *) malloc(sizeof(link_ll));
    l->data = data;
    l->previous = previous;
    l->next = next;
    return l;
}



double error_S(map *m,double **v){
    int i,j;
    double diff, maximum = 0;
    for(i=0;i<m->rows;i++){
        for(j=0;j<m->columns;j++){
            diff = fabs(m->cells[i][j].v-v[i][j]);
            if(diff > maximum){
                maximum = diff;
            }
        }
    }
    return maximum;
}

void compute_optimal_policy_S(S_MDP *mdp,map *m){
    int i,j;
    for(i=0;i<m->rows;i++){
        for(j=0;j<m->columns;j++){
            m->cells[i][j].pi =
            best_expected_action_S(mdp->succs[i*m->columns+j]);
        }
    }
}


void delete_grid(state **grid,int n){
    int i=0;
    for(i=0;i<n;i++){
        free(grid[i]);
    }
    free(grid);
}








void add_or_acc(S_MDP *mdp,map *m,int i,int j,int ip,int jp,int a,double prob){
    SP *sp;
    int found = 0;
    if(m->cells[ip][jp].type==INACCESS){
        //target cell inaccessible
        //agent remains in its location
        ip = i;
        jp = j;
    }
    for(link_ll *p=mdp->succs[i*m->columns+j][a]->first;p!=NULL;p=p->next){
        sp = (SP *)p->data;
        //state already in the list
        if(sp->s->index==ip*m->columns+jp){
            //update its probability
            sp->prob += prob;
            found = 1;
            break;
        }
    }
    if(!found){
        // create a new state probability pair
        sp = (SP *) malloc(sizeof(SP));
        sp->s = &(m->cells[ip][jp]);
        sp->prob = prob;
        // adds pair to list
        append(mdp->succs[i*m->columns+j][a],sp);
    }
}





double **allocate_Moore(){
    int i = 0;
    double** neigh = (double **) malloc(sizeof(double*)*3);
    for(i=0;i<3;i++){
        neigh[i] = (double *) malloc(sizeof(double)*3);
    }
    return neigh;
}

void deallocate_Moore(double **neigh){
    int i = 0;
    for(i=0;i<3;i++){
        free(neigh[i]);
    }
    free(neigh);
}

void release_S_MDP(S_MDP *mdp){
    for(int s=0;s<mdp->s;s++){
        for(int a=0;a<mdp->a;a++){
            free_data_LLST(mdp->succs[s][a]);
            free_LLST(mdp->succs[s][a]);
        }
        free(mdp->succs[s]);
    }
    free(mdp->succs);
    free(mdp);
}


S_MDP *allocate_SMDP(int s,int a, double gamma){
    int i,j;
    S_MDP *mdp = (S_MDP *) malloc(sizeof(S_MDP));
    mdp->s = s;
    mdp->a = a;
    mdp->gamma = gamma;
    mdp->succs = (LLST ***) malloc(sizeof(LLST **)*s);
    for(i=0;i<s;i++){
        mdp->succs[i] = (LLST **) malloc(sizeof(LLST *)*a);
        for(j=0;j<a;j++){
            // creates an empty list for (next state,probability) pairs
            mdp->succs[i][j] = empty();
        }
    }
    return mdp;
}


void wire_map(S_MDP *mdp,map *m,double ***probs){
    double prob;
    for(int i=0;i<m->rows;i++){
        for(int j=0;j<m->columns;j++){
            if(m->cells[i][j].type==INACCESS || m->cells[i][j].type==TERMINAL){
                //cell is inaccessible or terminal
                continue;
            }
            //actions
            for(int a=UP;a<=UP_LEFT;a++){
                // explore Moore's neighborhood
                for(int k=0;k<3;k++){
                    for(int l=0;l<3;l++){
                        prob = probs[a][k][l];
                        // if probability is not zero
                        if(prob){
                            add_or_acc(
                            mdp,
                            m,
                            i,
                            j,
                            (i+k-1<0)?0:(i+k-1)>=m->rows?m->rows-1:i+k-1,
                            (j+l-1<0)?0:(j+l-1)>=m->columns?m->columns-1:j+l-1,
                            a,
                            prob);
                        }
                        //else nothing to do
                    }
                }
            }
        }
    }
}
       

void v_iteration_S(S_MDP *mdp,map *m,double epsilon){
    int i,j;
    int k = 0;
    double **v, max_error;
    v = (double **) malloc(sizeof(double *)*m->rows);
    for(i=0;i<m->rows;i++){
        v[i] = (double *) calloc(m->columns,sizeof(double));
        for(j=0;j<m->columns;j++){
            v[i][j] = m->cells[i][j].reward;
            m->cells[i][j].v =v[i][j];
        }
    }
    do{
        for(i=0;i<m->rows;i++){
            for(j=0;j<m->columns;j++){
                if(m->cells[i][j].type==ACCESS){
                    v[i][j]=m->cells[i][j].reward+
                    mdp->gamma*best_expected_u_S(mdp->succs[i*m->columns+j]);
                }
            }
        }
        max_error = error_S(m,v);
        for(i=0;i<m->rows;i++){
            for(j=0;j<m->columns;j++){
                m->cells[i][j].v = v[i][j];
            }
        }
        k++;
        if(k > MDP_NUM_MAX_ITERATIONS) break;
    }while(max_error>(epsilon*(1-mdp->gamma)/mdp->gamma));
#ifdef DEBUG
    printf("\nKKK %d\n",k);
#endif
    compute_optimal_policy_S(mdp,m);
    // releasing allocated resources
    for(i=0;i<m->rows;i++){
        free(v[i]);
    }
    free(v);
}





state **create_empty_grid(int n, int m){
    int i = 0;
    state** grid = (state **) malloc(sizeof(state*)*n);
    for(i=0;i<n;i++){
        grid[i] = (state *) malloc(sizeof(state)*m);
    }
    return grid;
}



FILE *open_map(char *filename){
    FILE *fp = fopen(filename, "r");
    if (fp == NULL){
        printf("Error reading file: %s\n",filename);
        return NULL;
    }
    return fp;
}


map *read_problem(char *filename,double ***probs){
    map *m = (map *) malloc(sizeof(map));
    FILE *fp = open_map(filename);
    if(!fp){
        exit(1);
    }
    fscanf(fp,"%d %d\n",&(m->rows),&(m->columns));
    m->cells = create_empty_grid(m->rows,m->columns);
    for(int i=0;i<m->rows;i++){
        for(int j=0;j<m->columns;j++){
             fscanf(fp,"%d",&(m->cells[i][j].type));
             fscanf(fp,"%lf",&(m->cells[i][j].reward));
             m->cells[i][j].index = i*m->columns+j;
             m->cells[i][j].v = 0.0;
        }
    }
    for(int action = UP;action<=UP_LEFT;action++){
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                fscanf(fp,"%lf",&probs[action][i][j]);
            }
        }
    }
    fclose(fp);
    return m;
}


// mdpnav -g 1 -e 0.0008 -f 
void mdpnav_function(mdp_database mdp_stalin, char * input_file, char * output_file){
    char table[10][10];

    // the number of possible actions
    int actions = 8;
    // allocate space for probability distributions
    double ***probs = (double ***) malloc(sizeof(double **)*actions);



    for(int action = UP;action<=UP_LEFT;action++){
        probs[action] = allocate_Moore();
    }
    

    nav_spec p;
    // defaults
    p.epsilon=0.0008;
    p.gamma=0.99;
    
    //p.input_file = "map.mdp";
    //p.output_file = "policy.txt";
    // parse command line parameters
    //parse_params(argc,argv,&p);

    p.input_file= input_file;
    p.output_file= output_file;

    map * m = read_problem(p.input_file,probs);
    
    // create sparsed MDP
    S_MDP *mdp = allocate_SMDP( m->rows*m->columns/*states*/,
                                actions,/*number of actions*/
                                p.gamma/*gamma*/);
    wire_map(mdp,m,probs);
    // solve using value iteration
    v_iteration_S(mdp,m,p.epsilon);
    // write results
    report_file(m,p.output_file);
    
    // release resources
    delete_grid(m->cells,m->rows);
    for(int action = UP;action<=UP_LEFT;action++){
        deallocate_Moore(probs[action]);
    }
    free(probs);
    release_S_MDP(mdp);
    free(m);
    //return table;
}
