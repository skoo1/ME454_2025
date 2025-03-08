#include <iostream>
#include "include/listops.h"

using namespace std;

int main()
{
    // Declare and initialize the list structure
    MyList my_list;
    my_list.len = 0;
    
    // Variable to switch between two modes
    bool interact = true;

    if (!interact)
    {
        // Pre-set command mode (interact is false)
        // This part may be modified to test the functions

        append(&my_list, 3);
        append(&my_list, 4);
        append(&my_list, 5);
        display(&my_list);

        insert(&my_list, 1, 1);
        insert(&my_list, 1, 3);
        insert(&my_list, 1, 5);
        display(&my_list);

        cout << max(&my_list) << endl;

        insert(&my_list, 42, 4);
        insert(&my_list, 42, 0);
        display(&my_list);
        
        cout << max(&my_list) << endl;

        pop(&my_list, 0);
        pop(&my_list, 4);
        pop(&my_list, -1);

        cout << max(&my_list) << endl;

        display(&my_list);
        
    }
    else
    {
        // User interaction mode (interact is true)
        // This part does not need to be modified
        
        cout << "Hello ME454! Please type h for the help." << endl;

        char operation;
        char space;
        int args[MAX_ARGS];
        int idx_arg, num_arg, ret_val;
        
        // The while loop terminates until the input is 'q'

        while(true)
        {
            bool success = true;
            bool endline = false;

            // Read user command from the input stream

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

            // Call functions corresponding to the command

            if (operation == 'h' & idx_arg == 0 & success)
            {
                cout << "a [item] : adds [item] to the end of the list" << endl << "i [item] [index] : adds [item] to [index] of the list" << endl
                     << "m : finds the maximum item value in the list" << endl << "p [index] : removes the item at [index] (end of the list if [index] = -1) and returns it" << endl
                     << "q : exits from the program" << endl;
            }
            else if (operation == 'a' & idx_arg == 1 & success)
            {
                ret_val = append(&my_list, args[0]);
                if (ret_val == 0) cout << "Appended an item to the list" << endl;
                if (ret_val == -1) cout << "Could not append an item to the list" << endl;
                display(&my_list);
            }
            else if (operation == 'i' & idx_arg == 2 & success)
            {
                ret_val = insert(&my_list, args[0], args[1]);
                if (ret_val == 0) cout << "Inserted an item to the list" << endl;
                if (ret_val == -1) cout << "Could not insert an item to the list" << endl;
                display(&my_list);
            }
            else if (operation == 'm' & idx_arg == 0 & success)
            {
                ret_val = max(&my_list);
                if (ret_val == -1) cout << "Could not find the maximum value" << endl;
                else cout << "Item with the maximum value : " << ret_val << endl;
                display(&my_list);
            }
            else if (operation == 'p' & idx_arg == 1 & success)
            {
                ret_val = pop(&my_list, args[0]);
                if (ret_val == -1) cout << "Could not remove the item" << endl;
                else cout << "Removed the item : " << ret_val << endl;
                display(&my_list);
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

            // Reset the input stream

            cin.clear();
            if (!endline) cin.ignore(128, '\n');
        }
    }


    return 0;
}
