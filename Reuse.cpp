//#include "stdafx.h" // Visual Studio users need to uncomment this line
#include <iostream>

int checkoperatorSel(int v, int w)
{
    char operatorSel;
    int result;
    std::cout << "Enter operator as +-/*" << std::endl;
    std::cin >> operatorSel;
    bool validationNeeded = true;
    if (operatorSel == '+')
    {
        result = v + w;
        validationNeeded = false;
        std::cout << v << " + " << w;
    }
    if (operatorSel == '-')
    {
        result = 0;
        if (v != w || w != v)
        {
            result = v - w;
            validationNeeded = false;
            std::cout << v << " - " << w;
        }
        else if(v==w)
        {
            std::cout  << v << " - " << w << " you can't figure that out?" << std::endl;
            validationNeeded=true;
        }
    }
    if (operatorSel == '/')
    {
        result = v / w;
        validationNeeded = false;
        std::cout << v << " / " << w;
    }
    if (operatorSel == '*')
    {
        result = v * w;
        validationNeeded = false;
        if (v == 0 || w == 0)
        {
            validationNeeded = true;
        }
        std::cout << v << " * " << w;
    }
    if (validationNeeded == false)
    {
        std::cout << " = " << result << std::endl; // prints result if valid
    }
    if (validationNeeded == true)
    {
        std::cout << "\nhmmmm... that was a hard one!!" << std::endl; // doesn't print result if invalid
        return 0;
    }
}

int cont()
{
    bool exitProg = false;
    bool answered=false;
    while (answered == false)
    {
        char yORn;                                  // make char local
        std::cout << "Enter y or n to continue ";   // ask user for decision
        std::cin >> yORn;                           // get input from user
        if (yORn == 'y' || yORn == 'Y')             // checking routine y or n?
        {
            exitProg = true;
            answered = true; // make sure y or Y is entered
        }
        else if (yORn == 'n' || yORn == 'N')
        {
            exitProg = false;
            answered = true; // make sure n or N IS entered
        }
    }
    return exitProg; // return boolean yes or no
}
// getValueFromUser will read a value in from the user, and return it to the caller
int getValueFromUser()
{
    int a;                           // allocate a variable to hold the user input
    std::cout << "Enter a number: "; // ask user for an integer
    std::cin >> a;                   // get user input from console and store in variable a
    return a;                        // return this value to the function's caller (main)
}

int main()
{
    int exitHere = 1;
    while (exitHere == 1)
    {
        int x = getValueFromUser(); // first call to getValueFromUser
        int y = getValueFromUser(); // second call to getValueFromUser
        int result = checkoperatorSel(x, y);
        bool yn = cont(); // get user input from function boolean return
        if (yn == 1)
        {
            std::cout << "You are Continuing" << std::endl;
            exitHere = 1;
        }
        else if (yn != 1)
        {
            std::cout << "Bye" << std::endl;
            exitHere = 0;
        }
    }
    return 0;
}
