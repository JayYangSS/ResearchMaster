#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <cstdio>
#include <sstream>


using namespace std;

int main(int argc, char* argv[])
{
	ifstream src[100];

	ofstream dst;

	for (int i=0; i<argc-2; i++)
	{
		src[i].open(argv[i+2]);
	}

	dst.open(argv[1]);

	int j=0;
	string line;
	
	
	for (int i=0; i<argc-2; i++)
	{
		while (getline(src[i], line))
		{
				dst << line << endl;
				cout << j++ << endl;

				if(j > 40000)
					break;
		}
	}
	
		
	
	for (int i=0; i<argc-2; i++)
	{
		src[i].close();
	}

	dst.close();

	system("pause");
	return 0;
}