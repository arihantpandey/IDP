#include<bits/stdc++.h>
using namespace std;

int main()
{
    string file_in_name = "a.cpp";
    string file_out_name = "b.cpp";

    ifstream fileIn(file_in_name);
    ofstream fileOut(file_out_name);

    char c;

    while(true)
    {
        c = fileIn.get();
        if (c == EOF)
        {
            break;
        }

        if (c != '"')
        {
            fileOut.put(c);
        }
    }

    fileIn.close();
    fileOut.close();

    return 0;
}
