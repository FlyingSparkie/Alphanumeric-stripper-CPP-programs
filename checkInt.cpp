#include <iostream>
#include <string>

using namespace std;

int globalVar = 9999;

string checkit()
{
    string astring;
    cout << "Enter your alphanumeric string : ";
    cin >> astring;
    string holdString = astring;
    cout << holdString << endl;
    return holdString;
}

int getOdds(string &astring)
{
    int miscCount = 0;
    string str = astring;
    int acharVal;
    int length_of_string = str.size();
    for (int check_length = 0; check_length < length_of_string; check_length++)
    {
        bool found_non_char = false;
        acharVal = static_cast<char>(str[check_length]);
        if (acharVal <= 47 || acharVal >= 123)
        {
            found_non_char = true;
            std::cout << "Misc  : " << acharVal << " - is not alphanumeric " << str[check_length] << " position " << check_length << std::endl;
            miscCount++;
        }
        if (acharVal >= 58 && acharVal <= 64 && found_non_char != 1)
        {
            found_non_char = true;
            std::cout << "Misc  : " << acharVal << " - is not alphanumeric " << str[check_length] << " position " << check_length << std::endl;
            miscCount++;
        }
        if (acharVal >= 91 && acharVal <= 96 && found_non_char != 1)
        {
            found_non_char = true;
            std::cout << "M  : " << acharVal << " - is not alphanumeric " << str[check_length] << " position " << check_length << std::endl;
            std::cout << str[acharVal] << " - M " << miscCount << std::endl; // It's not a digit or alpha
            miscCount++;
        }
    }
    int outMisc = miscCount;
    std::cout << "\nfunction (getOdds) non alphanumberics counted : " << outMisc << std::endl;
    return outMisc;
}

int getNumbers(string &astring)
{
    string numberString;
    int digitCount = 0;
    string str = astring;
    int length_of_string = str.size();
    for (int index_pos = 0; index_pos < length_of_string; index_pos++)
    {
        if (isdigit(str[index_pos])) // checks string for digits in str(position[])
        {
            std::cout << str[index_pos] << " - D " << digitCount << " at position " << index_pos << std::endl; // It's a digit
            digitCount++;
            numberString += str.substr(index_pos, 1);
        }
    }
    int outDigits = digitCount;
    std::cout << "\nfunction (getnumbers) counted : " << outDigits << " numbers" << std::endl;
    if (outDigits >= 1)
    {
        std::cout << "Numbers extracted : " << numberString << std::endl;
    }
    if (outDigits == 0)
    {
        std::cout << "No numbers extracted" << std::endl;
    }
    return outDigits;
}
int getChars(string &bstring)
{
    string letterString;
    string str = "";
    int letterCount = 0;
    str = bstring;
    int length_of_string = str.size();
    for (int index_pos = 0; index_pos < length_of_string; index_pos++)
    {
        if (!isdigit(str[index_pos]))
        {
            std::cout << str[index_pos] << " - A " << letterCount << " at position " << index_pos << std::endl; // It's a letter
            letterCount++;
            letterString += str.substr(index_pos, 1);
        }
    }
    int outLetters = letterCount;
    cout << "\nfunction (getchars) counted " << outLetters << " chars" << endl;
    if (outLetters >= 1)
    {
        cout << "Chars extracted : " << letterString << endl;
    }
    else if (outLetters = 0)
    {
        cout << "No chars extracted" << endl;
    }
    return outLetters;
}

int main()
{
    std::cout << "This program is checkint.cpp" << endl;
    std::string UserInput = checkit();
    std::string Use_User_Input = UserInput;
    int chs = getChars(Use_User_Input);
    int nums = getNumbers(Use_User_Input);
    int misc = getOdds(Use_User_Input);
    std::cout << "your entry : " << UserInput << std::endl;
    if (nums >= 2)
    {
        std::cout << "\nThere are " << nums << " numbers" << std::endl;
    }
    else if (nums == 1)
    {
        std::cout << "\nThere is only one number" << std::endl;
    }
    else if (nums == 0)
    {
        std::cout << "No numbers were entered" << std::endl;
    }
    if (chs - misc >= 2)
    {
        std::cout << "There are " << chs - misc << " letters " << std::endl;
    }
    else if (chs - misc == 1)
    {
        std::cout << "There is only one letter " << std::endl;
    }
    else if (chs - misc == 0)
    {
        std::cout << "No letters were found" << std::endl;
    }
    if (misc >= 2)
    {
        std::cout << "There are " << misc << " that are neither letters or numbers " << std::endl;
    }
    else if (misc == 1)
    {
        std::cout << "There is only one that is neither letter or number" << std::endl;
    }
    else if (misc == 0)
    {
        std::cout << "There are no non number or characters found" << std::endl;
    }
}
