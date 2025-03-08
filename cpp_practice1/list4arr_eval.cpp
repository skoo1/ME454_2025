#include <iostream>
#include <string>
#include "include/listops.h"

using namespace std;

void test(int test_no, string test_name, int ans, int res)
{
    if (ans == res) cout << "[CLEAR] Test " << test_no << " : " << test_name << endl;
    else cout << "[     ] Test " << test_no << " : " << test_name << endl;
}

int main()
{
    // Declare and initialize list structure
    MyList my_list;
    my_list.len = 0;
    
    int test_clear = 0;
    int test_total = 0;
    
    test(1, "append return 1", append(&my_list, 2), 0);
    test(2, "append return 2", append(&my_list, -1), -1);
    test(3, "append function 1", my_list.items[0], 2);
    
    append(&my_list, 8);
    
    test(4, "append function 2", my_list.items[1], 8);

    test(5, "insert return 1", insert(&my_list, 7, 1), 0);
    test(6, "insert return 2", insert(&my_list, -1, 1), -1);
    test(7, "insert return 3", insert(&my_list, 0, 4), -1);
    test(8, "insert return 4", insert(&my_list, 0, -1), -1);
    test(9, "insert function 1", my_list.items[1], 7);
    test(10, "insert function 2", my_list.items[2], 8);

    test(11, "max return 1", max(&my_list), 8);

    test(12, "pop return 1", pop(&my_list, 1), 7);
    test(13, "pop return 2", pop(&my_list, 4), -1);
    test(14, "pop return 3", pop(&my_list, -1), -1);
    test(15, "pop function 1", my_list.items[1], 8);
    
    pop(&my_list, 0);
    pop(&my_list, 0);
    
    test(16, "pop function 2", my_list.len, 0);

    test(17, "max return 2", max(&my_list), -1);
    test(18, "pop return 4", pop(&my_list, 0), -1);

    for (int idx = 0; idx < LIST_CAPACITY - 1; idx++) append(&my_list, LIST_CAPACITY - idx);
    insert(&my_list, 42, LIST_CAPACITY - 1);

    test(19, "append return 3", append(&my_list, 0), -1);
    test(20, "insert function 3", my_list.items[LIST_CAPACITY - 1], 42);
    test(21, "insert return 5", insert(&my_list, 0, 0), -1);
    test(22, "max return 3", max(&my_list), LIST_CAPACITY);
    
    return 0;
}
