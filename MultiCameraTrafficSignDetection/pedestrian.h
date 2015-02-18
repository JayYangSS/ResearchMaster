


#include "stdafx.h"
using namespace cv;

class Pedestrian
{
	public:
		Rect locateR;
		Rect locateL;
		int misscount;
		bool posi;
		double dist;
		double angle;

		Pedestrian();
		virtual ~Pedestrian();
		void miss();
		void got();
		void draw(Mat imgl, Mat imgr, int t_color);
		void text(Mat imgl, Mat imgr, int t_color);
		string Pedestrian::DoubleToString(double value);
	protected:
    private:
};