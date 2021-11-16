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
 ******************************************************************************/
/**
 * @file dstruct.h
 * @author Stalin Muñoz Gutiérrez
 * @date 3 March 2019
 * @brief Data Structures Library
 *
 * Basic Data Structures Library Header File
 * 
 */
#ifndef _DSTRUCTH_
#define _DSTRUCTH_

/**
 * @brief the byte a bit is located in a BitArray
 */
#define byteofbit(x) (x>>3)
/**
 * @brief the bit location in a byte
 */
#define bitinbyte(x) (x%8)

// forward definition of link_ll
typedef struct link_ll link_ll; 

/**
 * @brief Doubly Linked List link_ll.
 */
struct link_ll{
    void *data; /**< pointer to the data element. */
    link_ll *next; /**< pointer to next element in this list. */
    struct link_ll *previous; /**< pointer to the previous element. */
};

/**
 * @brief Doubly Linked List.
 */
typedef struct LLST{
    unsigned long long size; /**< the size of the list. */
    link_ll *first; /**< the first element of the list. */
    link_ll *last; /**< the last element of the list. */
} LLST;

/**
 * @brief Array of bits
 */
typedef struct BitArray{
    unsigned long long size; /**< the number of bits. */
    unsigned char *bits; /**< the bit storage. */
} BitArray;

/**
 * @brief creates the new link_ll with the provided parameters
 * @param data datum to store in the link_ll
 * @param previous previous link_ll in the data structure
 * @param next next link_ll in the data structure
 */
link_ll *new_link_ll(void *data,link_ll *previous,link_ll *next);

/**
 * @brief appends an elemento to the link_lled list
 * @param list the link_lled list
 * @param data the element to append
 */
void append(LLST *list,void *data);

/**
 * @brief concatenates the second list in the first
 * deleting the memory of the second
 * @param a the first list
 * @param b the second list
 */
void join(LLST *a, LLST *b);

/**
 * @brief creates a shallow copy of the provided list
 * @param list the list to copy
 * @result the copy
 */
LLST *copy(LLST *list);

/**
 * @brief extracts the first element of the list (head)
 * @param the list
 * @result the pointer to the data,
 */
void *head(LLST *list);

/**
 * @brief empty list
 * @result a list with cero elements
 */
LLST *empty();

/**
 * @brief creates a new list with the concatenation of a and b
 * @param a first list
 * @param b second list
 * @result a new list with the concatenation of the arguments
 */
LLST *concat(LLST *a,LLST *b);

/**
 * @brief releses the memory of the list and its elements
 * but not of the contents of the list
 * @param list the list to release
 */
void free_LLST(LLST *list);
/**
 * @brief releases the memory of all data allocated in 
 * the elements of the list
 * @param list the list containing the data
 */
void free_data_LLST(LLST *list);

/**
 * @brief creates a new Array of bits of the given size
 * the array will be initialized with ceros
 * @param n the size of the array to allocate
 * @result a reference to the array
 */
BitArray *new_BitArray(unsigned long long n);

/**
 * @brief sets the bit at location pos
 * @param array the BitArray
 * @param bit the bit to set
 * @param pos the location of the bit in the array
 */
void set_bit(BitArray *array,unsigned char bit, unsigned long long pos);

/**
 * @brief extracts the bit at the given position in the BitArray
 * @param array the BitArray
 * @param pos the location of the bit in the array
 * @result the bit at the location
 */
unsigned char get_bit(BitArray *array,unsigned long long pos);

/**
 * @brief prints the contents of the bit array
 * @param array the array of bits to print
 */
void print_BitArray(BitArray *array);
/**
 * @brief releases the memory used by the array
 * @param array the array to release
 */
void free_BitArray(BitArray *array);

#endif
