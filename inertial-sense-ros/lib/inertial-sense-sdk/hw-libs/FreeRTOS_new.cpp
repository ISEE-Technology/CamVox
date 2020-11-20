/**
 * @file FreeRTOS_new.cpp
 * @author Yair Raz
 * @brief Implementation file for FreeRTOS overloaded new/delete
 *  	  operators.  This is needed keep the heap memory allocation
 *        task-safe (prevent task switching during allocation and 
 *        potential memory corruption.
 */


#include <new>
#include <cstdlib>
#include <compiler.h>

extern "C"	void *	pvPortMalloc(size_t xWantedSize);
extern "C"	void 	vPortFree(void * pv);

void * operator new(size_t size)
{
	void* ptr = pvPortMalloc(size);
	while (ptr == 0) {}
	return ptr;
}

void operator delete(void *p)
{
	vPortFree(p);
}

void * operator new[](size_t size)
{
	void* ptr = pvPortMalloc(size);
	while (ptr == 0) {}
	return ptr;
}

void operator delete[](void *p)
{
	vPortFree(p);
}


// These override the standard libc-nano malloc & free
// #pragma GCC push_options
// #pragma GCC optimize ("O0")

void *malloc( size_t size )
{
	return pvPortMalloc( size );
}

void free( void *ptr )
{
	vPortFree( ptr );
}

void *realloc(void *ptr, size_t size)
{	// Not available
	return nullptr;
}

void *calloc(size_t nitems, size_t size)
{	// Not available
	return nullptr;
}

// #pragma GCC pop_options

