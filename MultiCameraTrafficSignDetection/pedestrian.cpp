#include "pedestrian.h"
#include "stdafx.h"
static CvScalar colors[] = 
{
    {{0,255,0}},
	{{0,255,255}},
	{{0,128,255}},
	{{0,0,255}},
    {{255,0,255}},
	{{255,0,0}},
    {{255,128,0}},
    {{255,255,0}}
};

string Pedestrian::DoubleToString(double value) 
{
	int valueint = int (value);
	if (valueint == -2147483648)
		return "INF";
	char ss[20];
	sprintf(ss,"%.1lf",value);
	return ss;
}

Pedestrian::Pedestrian()
{
	misscount=0;
	posi=1;
}

Pedestrian::~Pedestrian(){};

void Pedestrian::draw(Mat imgl, Mat imgr, int t_color)
{
	if (posi)
	{
		rectangle(imgr, locateR.tl(), locateR.br(), colors[t_color%8], 2);
		rectangle(imgl, locateL.tl(), locateL.br(), colors[t_color%8], 2);
		circle(imgr, Point(locateR.x+locateR.width/2,locateR.y+locateR.height/2),6,colors[t_color%8],2,CV_FILLED);
		circle(imgl, Point(locateL.x+locateL.width/2,locateL.y+locateL.height/2),6,colors[t_color%8],1,CV_FILLED);
		circle(imgl, Point(locateL.x+locateL.width/2,locateL.y+locateL.height/2),6,colors[t_color%8],2,CV_FILLED);
	}
}

void Pedestrian::text(Mat imgl, Mat imgr, int t_color)
{
	if (posi)
	{
		putText(imgl,  DoubleToString(dist) + "m" , Point(locateL.x,(locateL.y-3 > 20 ? locateL.y-3 : 20)),FONT_HERSHEY_SIMPLEX, 0.55, colors[t_color%8], 1);
		putText(imgr,  DoubleToString(dist) + "m" , Point(locateR.x,(locateR.y-3 > 20 ? locateR.y-3 : 20)),FONT_HERSHEY_SIMPLEX, 0.55, colors[t_color%8], 1);
		putText(imgl,  DoubleToString(angle), Point(locateL.x+2,(locateL.y+17 > 40 ? locateL.y+17 : 40)),FONT_HERSHEY_SIMPLEX, 0.55, colors[t_color%8], 1);
		putText(imgr,  DoubleToString(angle), Point(locateR.x+2,(locateR.y+17 > 40 ? locateR.y+17 : 40)),FONT_HERSHEY_SIMPLEX, 0.55, colors[t_color%8], 1);
	}
}

void Pedestrian::miss()
{
	misscount++;
}

void Pedestrian::got()
{
	misscount=0;
	posi=1;
}
