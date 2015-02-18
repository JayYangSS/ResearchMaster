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
        vector<Rect>measureRectVector, predictRectVector;
		vector<Rect>::iterator measureItr,predictItr;
		int counter;
        bool init;
		Rect initRect;
		string warningStatus;
		string dangerStatus;
		string space;



        KalmanFilterTracker();
        //virtual ~KalmanFilterTracker();
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
		void changeStatus(Mat& img);
		//void putMeasurementText(Mat& img);
		//void changeMeasurementStatus(Mat& img);

    protected:
    private:
};
