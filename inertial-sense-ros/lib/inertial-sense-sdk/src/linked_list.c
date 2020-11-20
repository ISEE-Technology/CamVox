/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "linked_list.h"

// #define LL_VALIDATE_INPUT          // Uncomment to check that inputs are valid, not NULL.

void linkedListClear( linked_list_t *ll )
{
#ifdef LL_VALIDATE_INPUT
    // Validate Input
    if( ll==0 )
        return;
#endif

    ll->head = 0;
    ll->tail = 0;
}


void linkedListInsertAtHead( linked_list_t *ll, linked_list_node_t *newNode )
{
#ifdef LL_VALIDATE_INPUT
    // Validate Input
    if( ll==0 || newNode==0 )
        return;
#endif

    if( ll->head )
    {   // Non-empty linked list.  
        ll->head->prev = newNode;
        newNode->nextCt = ll->head;
        newNode->prev = 0;
        ll->head = newNode;
    }
    else
    {   // Empty linked list.  Add to head.
        ll->head = newNode;
        ll->tail = newNode;
    }
}


void linkedListInsertBefore( linked_list_t *ll, linked_list_node_t *node, linked_list_node_t *newNode )
{
    linked_list_node_t *prev;

#ifdef LL_VALIDATE_INPUT
    // Validate Input
    if( ll==0 || node==0 || newNode==0 )
        return;
#endif

    // Reference previous node
    prev = (linked_list_node_t*)(node->prev);

    // Prev <-> newNode
    if( prev )
    {   // Node is NOT head of linked list (prev exists).
        prev->nextCt = newNode;
        newNode->prev = prev;
    }
    else
    {   // Node is first.  Insert at head.
        ll->head = newNode;
        newNode->prev = 0;
    }

    // newNode <-> next node
    node->prev = newNode;
    newNode->nextCt = node;
}


void linkedListRemove( linked_list_t *ll, linked_list_node_t *node )
{
    linked_list_node_t* prev;
    linked_list_node_t* next;

#ifdef LL_VALIDATE_INPUT
    // Validate Input
    if( ll==0 || node==0 )
        return;
#endif

    // Reference adjacent nodes
    prev = (linked_list_node_t*)(node->prev);
    next = (linked_list_node_t*)(node->nextCt);

    // Update Head and Tail pointers if needed
    if( ll->head == node )  
        ll->head = next;
    
    if( ll->tail == node )  
        ll->tail = prev;

    // Remove item from linked list, connect adjacent nodes
    if(prev)    
        prev->nextCt = next;

    if(next)    
        next->prev = prev;
}


