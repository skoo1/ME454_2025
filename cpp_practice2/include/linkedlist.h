#ifndef LINKEDLIST_H_
#define LINKEDLIST_H_

#define MAX_ARGS 2
#define LIST_CAPACITY 64

// Variable to check memory management
extern int global_node_count;

struct Node
{
    int item;
    Node *next;

    // Code to check memory management
    Node() {global_node_count++;}
    ~Node() {global_node_count--;}
};

class MyLinkedList
{
    public:
        MyLinkedList();
        ~MyLinkedList();
        int at(int index);
        int append(int item);
        int insert(int item, int index);
        int max();
        int pop(int index);
        void display();

    private:
        int len;
        Node *head = NULL;
};

#endif