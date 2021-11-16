/*******************************************************************************
 *
 * Simple MDP Library,  
 * Copyright (c) 2018
 * Laboratorio de Biorobótica
 * https://biorobotics.fi-p.unam.mx/.
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
 * @file mdp.h
 * @author Stalin Muñoz Gutiérrez
 * @author Jesús Savage Carmona
 * @date March 2019
 * @brief Simple MDP library.
 *
 * Simple Markov Decision Process library header file
 * 
 */
 #ifndef _MDPH_
 #define _MDPH_
 #include "dstruct.h"
 #include <stdlib.h>
 #include <stdio.h>
 
 #define UP 0
 #define RIGHT_S 1
 #define DOWN 2
 #define LEFT 3
 #define UP_RIGHT 4
 #define DOWN_RIGHT 5
 #define DOWN_LEFT 6
 #define UP_LEFT 7
 
 #define INACCESS 0
 #define ACCESS 1
 #define TERMINAL 2
 
 //CODES for ARROWS
 extern char *ARROWS[8];
 
/**
 * @brief A Markov Decision Process.
 *
 * Structure containing all parameters, infinite horizon expected 
 * rewards and optimal policy.
 */
typedef struct{
  int s; /**< number of states*/
  int a; /**< number of actions*/
  double gamma; /**< discount factor */
  double ***P; /**< transition probabilities, 
              P(s'|s,a)(access: P[s'][s][a])*/
  unsigned char *t; /**< terminal states*/
  double *r; /**< the reward r(s)*/
  double *v; /**< the infinite horizon expected rewards*/
  int *pi; /**< the optimal policy*/
} MDP;

/**
 * @brief A Markov Decision Process.
 *
 * Sparse state structure containing all parameters, infinite horizon expected 
 * rewards and optimal policy.
 */
typedef struct{
  int s; /**< number of states*/
  int a; /**< number of actions*/
  double gamma; /**< discount factor */
  LLST ***succs; /**< array of array of list of successor states. [s][a]*/
} S_MDP;

/**
 * @brief A state of an MDP
 */
typedef struct{
    int index; /**< absolute index of the state*/
    int type; /**< 0 (not accessible), 1 (accessible) and 2 (terminal)*/
    double reward; /**< the reward of the state*/
    double v; /**< the infinite horizon expected reward*/
    int pi;
} state;

/**
 * Next state and its probability
 */
typedef struct{
    state *s; /**< next state*/
    double prob; /**< probability of reaching the state*/
} SP;

/**
 * @brief Mapa o grid de celdas
 */
typedef struct{
    int rows; /**< number of rows*/
    int columns; /**< number of columns*/
    state **cells; /**< celdas del mapa*/
} map;


/**
 * @brief Allocates memory for an MDP
 * @param s the number of states
 * @param a the number of actions
 * @param gamma the discount factor for future rewards
 * @result the MDP
 */
MDP *allocate_MDP(int s, int a, double gamma);

/**
 * @brief Allocates memory for an Sparsed MDP
 * @param s the number of states
 * @param a the number of actions
 * @param gamma the discount factor for future rewards
 * @result the Sparsed MDP
 */
S_MDP *allocate_SMDP(int s, int a, double gamma);

/**
 * @brief Releases memory for an sparsed MDP
 * @param the MDP to release
 * @result the memory is deallocated
 */
void release_S_MDP(S_MDP *mdp);

/**
 * @brief Releases memory for an MDP
 * @param the MDP to release
 * @result the memory is deallocated
 */
void release_MDP(MDP *mdp);
/**
 * @brief Prints the MDP parameters
 * @param mdp the MDP to print
 * @result prints MDP info to standard output
 */
void print_MDP(MDP *mdp);
/**
 * @brief Set random probabilities for the state transitions
 * @param mdp the MDP
 * @result probabilities are updated
 */
void random_MDP(MDP *mdp);

/**
 * @brief Compute v*(s) using value iteration
 * @param mdp the sparsed MDP
 * @param epsilon the maximum allowed error for the computation of v(s)
 * @result updates infinite horizon utilities and optimal policy
 */
void v_iteration_S(S_MDP *mdp,map *m,double epsilon);
/**
 * @brief Compute v*(s) using value iteration
 * @param mdp the MDP
 * @param epsilon the maximum allowed error for the computation of v(s)
 * @result updates infinite horizon utilities and optimal policy
 */
void v_iteration_compute(MDP *mdp, double epsilon);
/**
 * @brief Computes the best expected utility value for a given state
 * @param succs the state-probability pairs of a given state
 * @result the best expected utility value for the given state 
 * implicitly selected (pointer correspond to the selected state)
 */
double best_expected_u_S(LLST **succs);
/**
 * @brief Computes the best expected utility value for the state s
 * @param mdp the MDP
 * @param s the state for the computation
 * @result the best expected utility value for the state s
 */
double best_expected_u(MDP *mdp, int s);
/**
 * @brief Computes the action for the best expected utility value for a state
 * @param succs the successor of the selected state
 * @result the action that maximizes the expected utility value
 */
int best_expected_action_S(LLST **succs);
/**
 * @brief Computes the action for the best expected utility value for the state s
 * @param mdp the MDP
 * @param s the state for the computation
 * @result the action that maximizes the expected utility value
 */
int best_expected_action(MDP *mdp, int s);
/**
 * @brief Computes the expected utility value for a state when taking action a
 * @param succs the succesor state probability pairs for the selected state
 * @param a the action taken
 * @result the expected utility value
 */
double expected_u_S(LLST **succs,int a);
/**
 * @brief Computes the expected utility value for the state s when taking action a
 * @param mdp the MDP
 * @param s the state for the computation
 * @result the expected utility value
 */
double expected_u(MDP *mdp,int s, int a);
/**
 * @brief Computes the maximum error between the values of the MDP 
 * and the given current v's
 * @param m the map with the cells
 * @param v the infinite horizon utility values for the computation
 * @result the computed error
 */
double error_S(map *m,double **v);
/**
 * @brief Computes the maximum error between the values of the MDP and the given vector v
 * @param mdp the MDP
 * @param v the vector of values for the computation
 * @result the computed error
 */
double error(MDP *mdp,double *v);
/**
 * @brief Computes the optimal policy for the given MDP
 * @param mdp the sparsed MDP
 * @param m the map of cells
 * @result the optimal policy is updated for every cell in the map
 */
void compute_optimal_policy_S(S_MDP *mdp,map *m);
/**
 * @brief Computes the optimal policy for the given MDP
 * @param mdp the MDP
 * @result the optimal policy is updated for every state
*/
void compute_optimal_policy(MDP *mdp);
/**
 * @brief releases memory of the grid
 * @param grid pointer to the grid of cells
 * @param n the number of rows in the grid
 * @result memory is deallocated
 */
void delete_grid(state **grid,int n);
/**
 * @brief creates an empty grid with allocated memory
 * @param n number of rows
 * @param m number of columns
 * @result a pointer to the empty grid
 */
state **create_empty_grid(int n, int m);

/**
 * @brief allocates a 3X3 Moore neighborhood
 * @result a pointer to the neighborhood
 */
double **allocate_Moore();

/**
 * @brief deallocates the memory used by the neighbourhood
 * @param neigh the neighbourhood
 * @result the memory is released
 */
void deallocate_Moore(double **neigh);
/**
 * @brief opens the file
 * @param filename name of the file to read
 * @result a pointer to the file handler 
 */
FILE *open_map(char *filename);

/**
 * @brief reads the information of the problem description
 * @param filename name of the problem file
 * @param probs probabilities for a Moore neighborhood
 * This probabilities are considering for the following
 * action order N, NE, E, SE, S, SW, W, NW
 * @result a pointer to the map, Moore probabilities are loaded;
 */
map *read_problem(char *filename,double ***probs);

/**
 * @brief prepare the MDP with the map and state transition probabilites
 * @param mdp the Markov Decision Process
 * @param m the 2D map
 * @param probs the state transition probabilities of a Moore's neighborhood
 */
void wire_map(S_MDP *mdp,map *m,double ***probs);
/**
 * @brief absolute row
 * @param row the row of the origin cell
 * @param column the column of the origin cell
 * @param rows the number of rows in the map
 * @param columns the number of columns in the map
 * @param a the action taken
 * @param k the row in the neighborhood
 * @param l the column in the neighborhood
 * @result the absolute cell's row location in the map
 */
int absr(int row,int column,int rows,int columns,int a,int k,int l);

/**
 * @brief absolute column
 * @param row the row of the origin cell
 * @param column the column of the origin cell
 * @param rows the number of rows in the map
 * @param columns the number of columns in the map
 * @param a the action taken
 * @param k the row in the neighborhood
 * @param l the column in the neighborhood
 * @result the absolute cell's row location in the map
 */
int absc(int row,int column,int rows,int columns,int a,int k,int l);

/**
 * @brief add state or accumulates probability for a state probability pair
 * @param mdp the Markov Decision Proces
 * @param map the map
 * @param i the row of the source state
 * @param j the column of the source state
 * @param ip the row of the target state
 * @param jp the column of the target state
 * @param a the action taken
 * @param prob the probability of reaching the state
 * @result the state probability pair is added or updated
 */
void add_or_acc(S_MDP *mdp,map *m,int i,int j,int ip,int jp,int a,double prob);

#endif
