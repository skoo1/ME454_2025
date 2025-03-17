#include <iostream>
#include "include/linkedlist.h"

using namespace std;

int main()
{
    // Declare and initialize list structure
    MyLinkedList my_list;
    
    bool interact = true;

    if (!interact)
    {
        // Pre-set command mode
        // This part may be modified to test the functions

        my_list.append(3);
        my_list.append(4);
        my_list.append(5);
        my_list.display();

        my_list.insert(1, 1);
        my_list.insert(1, 3);
        my_list.insert(1, 5);
        my_list.display();

        cout << my_list.max() << endl;

        my_list.insert(42, 4);
        my_list.insert(42, 0);
        my_list.display();
        
        cout << my_list.max() << endl;

        my_list.pop(0);
        my_list.pop(4);

        cout << my_list.max() << endl;

        my_list.display();
        
    }
    else
    {
        // User interaction mode
        // This part does not need to be modified
        
        cout << "Hello ME454! Please type h for the help." << endl;

        char operation;
        char space;
        int args[MAX_ARGS];
        int idx_arg, num_arg, ret_val;

        while(true)
        {
            bool success = true;
            bool endline = false;
            cin >> operation;
            // cout << "operation : " << operation << endl;

            if (operation == 'a' | operation == 'p') num_arg = 1;
            else if (operation == 'i') num_arg = 2;
            else num_arg = 0;

            for (idx_arg = 0; idx_arg < num_arg; idx_arg++)
            {
                cin.read(&space, 1);
                if (space == '\n')
                {
                    endline = true;
                    break;
                } 
                cin >> args[idx_arg];
                if (cin.fail())
                {
                    success = false;
                    break;
                }
                // cout << "argument " << idx_arg << " : " << args[idx_arg] << endl;
            }

            if (operation == 'h' & idx_arg == 0 & success)
            {
                // help operation
                cout << "a [item] : adds [item] to the end of the list" << endl << "i [item] [index] : adds [item] to [index] of the list" << endl
                     << "m : finds the maximum item value in the list" << endl << "p [index] : removes the item at [index] (end of the list if [index] = -1) and returns it" << endl
                     << "s : sorts the list in ascending order" << endl << "q : exits from the program" << endl;
            }
            else if (operation == 'a' & idx_arg == 1 & success)
            {
                ret_val = my_list.append(args[0]);
                if (ret_val == 0) cout << "Appended an item to the list" << endl;
                if (ret_val == -1) cout << "Could not append an item to the list" << endl;
                my_list.display();
            }
            else if (operation == 'i' & idx_arg == 2 & success)
            {
                ret_val = my_list.insert(args[0], args[1]);
                if (ret_val == 0) cout << "Inserted an item to the list" << endl;
                if (ret_val == -1) cout << "Could not insert an item to the list" << endl;
                my_list.display();
            }
            else if (operation == 'm' & idx_arg == 0 & success)
            {
                ret_val = my_list.max();
                if (ret_val == -1) cout << "Could not find the maximum value" << endl;
                else cout << "Item with the maximum value : " << ret_val << endl;
                my_list.display();
            }
            else if (operation == 'p' & idx_arg == 1 & success)
            {
                ret_val = my_list.pop(args[0]);
                if (ret_val == -1) cout << "Could not remove the item" << endl;
                else cout << "Removed the item : " << ret_val << endl;
                my_list.display();
            }
            else if (operation == 'q' & idx_arg == 0 & success)
            {
                cout << "Terminating the program." << endl;
                break;
            }
            else
            {
                success = false;
            }

            if (!success) cout << "Could not successfully read the command. Please type h for the help." << endl << endl;

            cin.clear();
            if (!endline) cin.ignore(128, '\n');
        }
    }


    return 0;
}
