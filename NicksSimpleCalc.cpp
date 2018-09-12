//#include "stdafx.h" // Visual Studio users need to uncomment this line
#include <iostream>
#include <string>

using namespace std;

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
        else if (v == w)
        {
            std::cout << v << " - " << w << " you can't figure that out?" << std::endl;
            validationNeeded = true;
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
    bool answered = false;
    while (answered == false)
    {
        char yORn;                         // make char local
        std::cout << "Continue Y or N - "; // ask user for decision
        std::cin >> yORn;                  // get input from user
        if (yORn == 'n' || yORn == 'N')    // checking routine y or n?
        {
            exitProg = true;
            answered = true; // make sure y or Y is entered
        }
        else if (yORn == 'y' || yORn == 'Y')
        {
            exitProg = false;
            answered = true; // make sure n or N IS entered
        }
    }
    return exitProg; // return boolean yes or no
}
// getValueFromUser will read a value in from the user, and return it to the caller
int getValueFromUser(int nextNum)
{
    string a;
    //int a;
    bool tryAgain = false;
    //while (tryAgain == false)
    {
        // int nextNum;
        // int a; // allocate a variable to hold the user input
        //std::cout << "Enter a number: "; // ask user for an integer

        if (nextNum == 0)
        {
            std::cout << "Enter a number " << std::endl;
        }
        else if (nextNum == 1)
        {
            std::cout << "Enter another number " << std::endl;
        }
        std::cin >> a; // get user input from console and store in variable a

        //int x= cin.((a));
        //r (int t=0; t<x;t++){
        //   cout << x << " , " << t << std::endl;
        std::string str = a;
        std::cout << str << ":" << str.size() << std::endl;
       int stoi(const string& a, size_t *idx = 0, int base = 10);
       std::cout << " to integer " << stoi << std::endl;;

        return 0;

       /* while (tryAgain)
        {
            if (std::cin.fail())
            {
                std::cout << " is not a number " << std::endl;
                tryAgain = true;
                //a = 0;
                std::cin.clear();
                for (int i = 0; i < 20; i++)
                {
                    std::cout << i;
                }
                std::cout << "enter  a  number   ";
                std::cin >> a;
            }
        }*/
    }
    //checkNum(a);
   // if (tryAgain == false)
    {
        return true;
    }
}

int main()
{
    int nextNum = 0;
    int exitHere = 0;
    while (exitHere == 0)
    {
        nextNum = 0;
        int x = getValueFromUser(nextNum); // first call to getValueFromUser
                                           // checkNum(x);
        nextNum = 1;
        int y = getValueFromUser(nextNum); // second call to getValueFromUser
        int result = checkoperatorSel(x, y);
        bool yn = cont(); // get user input from function boolean return
        if (yn == 0)
        {
            std::cout << "You are Continuing" << std::endl;
            exitHere = 0;
        }
        else if (yn != 0)
        {
            std::cout << "Bye" << std::endl;
            exitHere = 1;
        }
    }
    return 0;
}

/*         if (y == 99)
        {
            std::cout << y << "! Enter a numeric value !" << std::endl;
        }

                if (x == 99)
        {
            std::cout << x << "!  Numerical value not entered  !" << std::endl;
        }

        */
/*         if (!isdigit(a))
        {
            std::cout << a << "NOT a numeric number" << std::endl;
            return c=false;
        }
        else if (isdigit(a))
        {
            return c==true; // return this value to the function's caller (main)
        }
        */
/*void check_valid(const char *val)
{
    std::locale loc(std::locale::empty(), std::locale::classic(), std::locale::numeric);
    for (size_t i = 0; i < strlen(val); i++)
    {
        char c = val[i];
        if (isdigit(c))
            throw std::invalid_argument("alpha character found in numeric_cast");
    }
}
*/
/*    if (!std::cin) // or if(cin.fail())
    {
        // user didn't input a number
        std::cin.clear();                                                   // reset failbit
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); //skip bad input
        // next, request user reinput
    }
    */
/*
   int checkNum(char a)
{
    bool ok;
    bool returnNum;
    while (!ok)
    {
        if (isdigit(a))
        {
            ok = true;
            std::cout << "Number entered correct";
            returnNum = a;
        }
        else
        {
            std::cout << "Incorrect number";
            ok = false;
        }
    }
    return returnNum;
}*/