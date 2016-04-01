#include "FileTan.h"


CFileTan::CFileTan(void)
{

}


CFileTan::~CFileTan(void)
{
}


bool CFileTan::readFile(char* fileName)
{
	sprintf(myfilename,"%s",fileName);
	inFile.open(fileName,ios::in);
	if(!inFile)     
		return false;  
	return true;
}

string CFileTan::GetValue(string key)
{
	string str,skey,ret;
	skey=key+"=";
	inFile.close();
	inFile.open(myfilename,ios::in);

	while(getline(inFile, str))     
	{         
		//process
		int iff=str.find( skey );
		if( iff == 0 )
		{
			ret=str.substr(skey.length());
			break;
		}
	}
	return ret;
}
