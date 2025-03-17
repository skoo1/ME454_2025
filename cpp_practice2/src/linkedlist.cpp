#include <iostream>
#include "../include/linkedlist.h"

using namespace std;

int global_node_count = 0;

// Initializes an object
MyLinkedList::MyLinkedList()
{
    len = 0;
    head = new Node;
    // Node new_node;
    // head = &new_node;
    head->item = -1;
    head->next = NULL;
}

// Initializes an object
MyLinkedList::~MyLinkedList()
{
    for (int i_link = 0; i_link < len; i_link++) pop(len - i_link - 1);
    delete head;
}

// Returns an item in the specific position (index) of the list 
// Returns -1 if 1) the index is negative 2) the index is not smaller than the length
int MyLinkedList::at(int index)
{
    // return -1 for the two cases
    if (index < 0) return -1;
    if (len <= index) return -1;

    // move to a node at the position through the pointer
    Node *curr = head;
    for (int i_link = 0; i_link < index; i_link++) curr = curr->next;

    return curr->next->item;
}


// Adds an item to the end of the list 
// Returns 0 if the element is successfully added
// Returns -1 if the item is negative
int MyLinkedList::append(int item)
{
    // return -1
    if (item < 0) return -1;

    // move to the tail node of the list
    Node *curr = head;
    for (int i_link = 0; i_link < len; i_link++) curr = curr->next;

    // add the item to the list and increase the length
    curr->next = new Node;
    curr->next->item = item;
    curr->next->next = NULL;
    len++;  

    return 0;
}

// Adds an item to the specific position (index) of the list
// Returns 0 if the element is successfully added
// Returns  -1 if 1) the item is negative, 2) the index is negative, or 3) the index is bigger than the length
int MyLinkedList::insert(int item, int index)
{
    return -1;
}

// Returns the biggest item in the list
// Returns -1 if the list is empty
int MyLinkedList::max()
{
    return -1;
}

// Removes and returns an item from the specific position (index) of the list
// Returns -1 if 1) the index is negative 2) the index is not smaller than the length
int MyLinkedList::pop(int index)
{
    // return -1 for the two cases
    if (index < 0) return -1;
    if (len <= index) return -1;

    // move to the position of the list
    Node *curr = head;
    for (int i_link = 0; i_link < index; i_link++) curr = curr->next;
    
    // remove the item and decrease the length
    Node *temp = curr->next;
    int pop_val = temp->item;
    curr->next = curr->next->next;
    delete temp; // delete the node to free memory
    len--;

    return pop_val;
}

// Displays the list 
void MyLinkedList::display()
{
    Node *curr = head;
    cout << "[ ";
    for (int i_link = 0; i_link < len; i_link++)
    {
        curr = curr->next;
        cout << curr->item << ", ";
    }
    cout << "]" << endl << endl;
}
