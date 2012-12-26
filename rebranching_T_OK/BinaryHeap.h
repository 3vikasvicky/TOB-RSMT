#ifndef _BINARYHEAP_H_
#define _BINARYHEAP_H_ 

#include <string>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <exception>
#include <sstream>
using namespace std;

template <class T,class E> struct HeapNode
{
	E element ;
	T tag ;
};
template <class T,class E> class BinaryHeap
{
private:
	int capacity ;
	int size ;
	struct HeapNode < T, E > * v ;//only use v[1]~v[capacity]
public:
	BinaryHeap () ;
	BinaryHeap (int) ;
    ~BinaryHeap () ;
    BinaryHeap (const BinaryHeap<T,E>& that);
    BinaryHeap<T,E>& operator = (const BinaryHeap<T,E>& that);
    void copy (const BinaryHeap<T,E>& that);
	void ensureCapacity (int) ;
	int getSize () const;
	bool isEmpty () ;
	string toString () ;
	int put (T, E) ;
	E min () ;
	T minTag () ;
	E extractMin () ;
	bool decrease (T newtag, E u) ;
    int findElement(E);
    T getTag(E);

};

template <class T, class E> BinaryHeap< T, E >::BinaryHeap () 
{
	capacity = 8 ;
	size = 0 ;
	v = new HeapNode < T, E > [capacity+1] ;

}

template <class T, class E> BinaryHeap< T, E >::BinaryHeap (int cap):capacity (cap) 
{
	size = 0 ;
	v = new HeapNode < T, E > [capacity+1] ;

}

template <class T, class E> BinaryHeap< T, E >::~BinaryHeap() {
    delete [] v ;
}


template <class T, class E> bool BinaryHeap< T, E >::decrease (T newtag,  E u)//only decrease the one existing in heap, and with smaller newTag
{
	int index ;
    int j;
	for (j = 1; j <=size; j++)
	{
		if (v[j].element == u)
			index = j;
	}
    if (j == size+1)    return false;

    if (newtag >= v[index].tag)  return false;

	v[index].tag = newtag ;
	int i = index;
    if (i > 1){
        while ( v[i].tag < v [(int) (i / 2)].tag && i > 1 )
        {
            HeapNode < T, E > tempNode = v[(int) (i / 2)];
            v [(int) (i / 2)] =  v [i] ;
            v [ i ] = tempNode ;
            i = (int) (i / 2) ;
        }
    }
    return true;
} 

template <class T, class E> int BinaryHeap< T, E >::findElement (E u)//only decrease the one existing in heap, and with smaller newTag
{
    int j;
	for (j = 1; j <=size; j++)
	{
		if (v[j].element == u)
            return j;
	}
    return -1;
} 
template <class T, class E> T BinaryHeap< T, E >::getTag (E u)//only decrease the one existing in heap, and with smaller newTag
{
    int j;
	for (j = 1; j <=size; j++)
	{
		if (v[j].element == u)
            return v[j].tag;
	}
} 

template <class T, class E>
BinaryHeap< T, E >::BinaryHeap (const BinaryHeap<T,E>& that){
    copy(that);
}
template <class T, class E>
BinaryHeap<T,E>& BinaryHeap< T, E >::operator = (const BinaryHeap<T,E>& that){
    if (this==&that)    return *this;
    copy(that);
    return *this;
}
template <class T, class E>
void BinaryHeap< T, E >::copy (const BinaryHeap<T,E>& that){
    delete [] v;
	v = new HeapNode < T, E > [that.capacity];
	for (int i = 1; i <= that.size ; i ++ )
	{
		v[i] = that.v[i];
	}
	capacity = that.capacity;
    size = that.size;

}

template <class T, class E> void BinaryHeap< T, E >::ensureCapacity (int cap) 
{
	HeapNode < T, E > * newV = new HeapNode < T, E > [cap+1] ;
	for (int i = 1; i <= capacity ; i ++ )
	{
		newV[i] = v[i] ;
	}
	capacity = cap ;
	delete [] v ;
	v = newV ;
}

template <class T, class E> int BinaryHeap< T, E >::getSize ()const 
{
	return size ;
}

template <class T, class E> bool BinaryHeap< T, E >::isEmpty () 
{
	return (size == 0) ;
}

template <class T, class E> string BinaryHeap< T, E >::toString () 
{
	string s = "";
	std::ostringstream oss;
	for (int i = 1; i <= size; i++ )
	{
		oss << v[i].tag << " -> " << v[i].element << endl ;
	}
	s = oss.str() ;
	return s; 
}

template <class T, class E> int BinaryHeap< T, E >::put (T tag, E element) 
{
    int j = findElement(element);
    if (j != -1)    return j;
	if (size == capacity)
	{
		//exception RuntimeException ;
		//throw RuntimeException ;
        ensureCapacity(2*capacity);
	}
	size = size + 1;
	HeapNode < T, E> newNode;
	newNode.element = element ;
	newNode.tag = tag ;
	v[size] = newNode ;
	int i = size ;
    if ( size > 1){
        while ( v[i].tag < v [(int) (i / 2)].tag && i > 1 )
        {
            HeapNode < T, E > tempNode = v[(int) (i / 2)];
            v [(int) (i / 2)] =  v [i] ;
            v [ i ] = tempNode ;
            i = (int) (i / 2) ;
        }
    }
    return i ;
}

template <class T, class E> E BinaryHeap< T, E >::min () 
{
	
	if (size == 0 )
	{
		exception NoSuchElementException ;
		throw NoSuchElementException ;
	}
	return v[1].element ;
}

template <class T, class E> T BinaryHeap< T, E >::minTag () 
{

	if (size == 0 )
	{
		exception NoSuchElementException ;
		throw NoSuchElementException ;
	}
	return v[1].tag ;
}

template <class T, class E> E BinaryHeap< T, E >::extractMin () 
{

	if (size == 0 )
	{
		exception NoSuchElementException ;
		throw NoSuchElementException ;
	}
	E returnElement = v[1].element ;
	v[1] = v[size] ;
	size = size - 1;

	int i = 1;
	while ( i <= size )
	{
		int smallest = i ;
		if (2 * i + 1 <= size && v[2*i + 1].tag <= v[i].tag )
			smallest = 2 * i + 1;
		
		if (2 * i  <= size && v[2*i].tag <= v[smallest].tag )
			smallest = 2 * i ;

		if (smallest == i)
			return returnElement ;

		HeapNode <T, E > tempNode = v[i] ;
		v[i] = v[smallest] ;
		v[smallest] = tempNode;
		i = smallest ;
	}
	return returnElement;
}



#endif
