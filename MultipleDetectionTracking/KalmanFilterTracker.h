#include "stdafx.h"

using namespace cv;
using namespace std;

class KalmanFilterTracker
{
    public:
        KalmanFilter KF;
        Mat statePostMat;
		Mat statePreMat;        
        Mat measurement;
		Mat precessNoise;
        vector<Rect>measureRectVector, statePostRectVector, statePreRectVector;
		vector<Rect>::iterator measureItr,predictItr;
		Rect measurementRect;
		Rect statePostRect;
		Rect statePreRect;
		int counter;
        bool init;
		Rect initRect;
		string warningStatus;
		string dangerStatus;
		string space;



        KalmanFilterTracker();
        //~KalmanFilterTracker();
        void updateKF(int x, int y, int width, int height);		//input the measurement states
        void initKF(int x, int y, int width, int height);
		void updateWithoutMeasurement();

		void addcounter();
		int getcounter();
		void drawPredictLine(Mat& img);
		void drawMeasureLine(Mat& img);
		void drawPretictLinelns(Mat img, double ratio, int resx, int resy, int tempx, int tempy);
		void drawMeasureLinelns(Mat img, double ratio, int resx, int resy, int tempx, int tempy);
		void putStatusText(Mat& img);		
		void changeStatusImgL(Mat& img);
		void changeStatusImgS(Mat& img);
		//void putMeasurementText(Mat& img);
		//void changeMeasurementStatus(Mat& img);

    protected:
    private:
};
