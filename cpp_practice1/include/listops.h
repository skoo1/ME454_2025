#ifndef LISTOPS_H_
#define LISTOPS_H_

#define MAX_ARGS 2
#define LIST_CAPACITY 64

struct MyList
{
    int len;
    int items[LIST_CAPACITY];
};

int append(MyList *p_list, int item);
int insert(MyList *p_list, int item, int index);
int max(MyList *p_list);
int pop(MyList *p_list, int index);
void display(MyList *p_list);

#endif