/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef LINKED_LIST_H
#define LINKED_LIST_H

#ifdef __cplusplus
extern "C" {
#endif


//_____ D E F I N I T I O N S ______________________________________________

typedef struct          // Place this structure at the start of any data structure to reference properly
{
    void                *prev;                 // Next object in linked list.  0 indicates this is the head.
    void                *nextCt;                 // Prev object in linked list.  0 indicates this is the tail.
}linked_list_node_t;

typedef struct
{
    linked_list_node_t  *head;                  // Head of linked list
    linked_list_node_t  *tail;                  // Tail of linked list
}linked_list_t;

//_____ P R O T O T Y P E S ________________________________________________

void linkedListClear( linked_list_t *ll );
void linkedListInsertAtHead( linked_list_t *ll, linked_list_node_t *newNode );
void linkedListInsertBefore( linked_list_t *ll, linked_list_node_t *node, linked_list_node_t *newNode );
void linkedListRemove( linked_list_t *ll, linked_list_node_t *node );


#ifdef __cplusplus
}
#endif

#endif // LINKED_LIST_H
