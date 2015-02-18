#include <stdio.h>
#include <tchar.h>
#include <cstdlib>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <algorithm>
#include <cmath>

const double PI = 3.1415926;
using namespace std;

double getVerticalFOV(double ImgV, double HogV, double Dis)
{
	double resultFOV = 2 * atan( (ImgV*1.6) / (2*Dis*HogV) ) * 180/PI;
	return resultFOV;
}

double getFocalLength(double verticalFOV)
{
	double temp = verticalFOV * PI/180;
	double resultFL = 3.6/( 2*tan( temp/2 ) );
	return resultFL;
}

double getMy(double vFL_S, double vFL_L, double ImgV)
{
	double My = ImgV/2 - (vFL_S*ImgV)/(vFL_L*2);
	return My;
}

double getMx(double vFL_S, double vFL_L, double ImgH)
{
	double Mx = ImgH/2 - (vFL_S*ImgH)/(vFL_L*2);
	return Mx;
}

double getMinDD(double vFOV_S, double HogV, double ImgV)
{
	double temp = (vFOV_S/2) * PI/180;
	double MinDD = max(1.4/tan(temp), (ImgV*1.6)/(2*tan(temp)*ImgV) );
	return MinDD;
}

double getMaxDD(double vFOV_L,double HogV, double ImgV)
{
	double temp = (vFOV_L/2) * PI/180;
	double MaxDD = (ImgV*1.6)/(2*tan(temp)*HogV);
	return MaxDD;
}

int main(int argc, char** argv)
{

	double ImgH = 800;
	double HogH = 48;
	double Dis = 35;
	if(argc != 1 &&argc != 4)
	{
		cout<<"double ImgH, double HogH, double Dis "<<endl;
	}
	if(argc == 4)
	{
		ImgH = *argv[1];
		HogH = *argv[2];
		Dis = *argv[3];
	}
	double ImgV = ImgH*3/4;
	double HogV = HogH*2;

	double vFOV_S, vFL_S, MinDD, MaxDD, Mx, My;
	double vFOV_L,vFL_L;
	if(Dis == 15)
	{
		vFOV_L = 11;
		vFL_L = 18.7;
	}
	if(Dis == 35)
	{
		vFOV_L = 4.6;
		vFL_L = 45;
	}
	if(Dis == 80)
	{
		vFOV_L = 2;
		vFL_L = 103;
	}


	vFOV_S = getVerticalFOV(ImgV, HogV, Dis);
	vFL_S = getFocalLength(vFOV_S);
	MinDD = getMinDD(vFOV_S,HogV,ImgV);
	MaxDD = getMaxDD(vFOV_L,HogV,ImgV);
	Mx = getMx(vFL_S,vFL_L,ImgH);
	My = getMy(vFL_S,vFL_L,ImgV);

	cout<<" ImgV  "<<ImgV<<" HogV "<<HogV<<" Dis "<<Dis<<endl;
	cout<<"vFOV_S is "<<vFOV_S<<"; vFL_S is  "<<vFL_S<<endl;
	cout<<"Mx "<<Mx<<"; My  "<<My<<endl;
	cout<<"MinDD "<<MinDD<<"; MaxDD  "<<MaxDD<<endl;

	system("pause");
	return 0;
}