#pragma once
#include <iostream>
#include <fstream>
#include <string> 
using namespace std; 
class CFileTan
{
private:
	ifstream inFile; 
public:
	CFileTan(void);
	~CFileTan(void);
	bool readFile(char* fileName);
	string GetValue(string key);
};

