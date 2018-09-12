#include <iostream>
#include <string>
//string a;
using namespace std;

int checkit(int& b, int& c)
{
    string astring;
    std::cout << "Enter your number : ";
    std::cin >> astring;
    const string hh = astring;
    int digitCount = 0;
    string str = astring;
    int length_ofstring = str.size();
    for (int checking = 0; checking < length_ofstring; checking++)
    {
        if (isdigit(str[checking]))
        {
            std::cout << "D " << digitCount << std::endl; // It's a digit
            digitCount++;
        }
    }
    b = digitCount;
    int letterCount = 0;
    for (int checking = 0; checking < length_ofstring; checking++)
    {
        if (!isdigit(str[checking]))
        {
            std::cout << "A " << letterCount << std::endl; // It's a letter
            letterCount++;
        }
    }
    c = letterCount;
    return b,c;
   // std::cout << "\n letters " << c << std::endl;
    //return b, c;
}

int main()
{
    std::cout << "This is program checkint2" << std::endl;
    int c = 0;
    int b = 0;
    checkit(c, b);

    std::cout << "numbers = : " << c << " letters = : " << b << std::endl;
}
// int b = getchars(aa);
//int c = getnumbers(aa);
//std::cout << "\nThere are " << c << " numbers" << std::endl;
//std::cout << "There are " << b << " letters " << std::endl;
//std::cout << num_of_numbers << "   :   " << num_of_letters << std::endl;
//string astring;
//checkit(astring);
//string str = astring;
//int length_ofstring = str.size();
//string a;
//int b = 0;
//int c = 0;

//checkit(num_of_numbers, num_of_letters);