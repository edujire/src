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

#include "mdp.h"
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<unistd.h>

#ifndef _MDPNAVH_
#define _MDPNAVH_
/**
 * @brief problem parameters
 */
typedef struct{
    char *input_file; /**< name of the input file */
    char *output_file; /**< name of the output file */
    double epsilon; /**< maximum error */
    double gamma; /**< discout factor */
} nav_spec;

/**
 * @brief writes the result to a file
 * @param m the map of cells
 * @param output_file name of the output file
 * @result the output is writen to the file
 */ 
void report_file(map *m,char *output_file);
/**
 * @brief parses command line arguments
 * @param argc number of command line arguments
 * @param argv command line arguments
 * @param p pointer to the parameter simulation structure
 */
void parse_params(int argc, char *argv[], nav_spec *p);
#endif
