#include <iostream>
#include "../include/listops.h"

using namespace std;

// Adds an item to the end of the list 
// Returns 0 if the element is successfully added
// Returns -1 otherwise: 1) if the list is full, or 2) the item is negative
int append(MyList *p_list, int item)
{
    // return -1 for the two cases
    if (p_list->len >= LIST_CAPACITY) return -1; 
    if (item < 0) return -1;

    // add the element to the list and increase the length
    p_list->items[p_list->len] = item;
    p_list->len++;

    return 0;
}

// Adds an item to the specific position (index) of the list
// Returns 0 if the element is successfully added
// Returns -1 otherwise: 1) the list is full, 2) the item is negative, 3) the index is negative, or 4) the index is bigger than the length
int insert(MyList *p_list, int item, int index)
{
    return -1;
}

// Returns the biggest item in the list
// Returns -1 if the list is empty
int max(MyList *p_list)
{
    return -1;
}

// Removes and returns an item from the specific position (index) of the list
// Returns -1 if 1) the index is negative 2) the index is not smaller than the length
int pop(MyList *p_list, int index)
{
    return -1;
}

// Displays the list 
void display(MyList *p_list)
{
    cout << "[ ";
    for (int i_arr = 0; i_arr < p_list->len; i_arr++)
    {
        cout << p_list->items[i_arr] << ", ";
    }
    cout << "]" << endl << endl;
}
