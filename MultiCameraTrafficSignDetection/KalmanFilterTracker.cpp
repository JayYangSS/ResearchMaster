
#include "stdafx.h"

using namespace cv;
using namespace std;

const int stateNum = 8;										//x,y,width,height,detax,detay,deta width,deta height
const int measureNum = 4;									//detax,detay,deta width,deta height

KalmanFilterTracker::KalmanFilterTracker()
{
    KF = KalmanFilter(stateNum, measureNum, 0);
    statePostMat = Mat (stateNum, measureNum, CV_32F);
	statePreMat = Mat(measureNum, 1, CV_32F);
    precessNoise = Mat(stateNum, 1, CV_32F);
    measurement = Mat (measureNum, 1, CV_32F);
	measurement.setTo(Scalar::all(0));
    init = false;
	
}

void KalmanFilterTracker::initKF(int x, int y, int width, int height) 
{
	cout<<"KalmanFilterTrackerInitialization"<<endl;
	if(!measureRectVector.empty() )
	{
		measureRectVector.clear();	
		cout << "clear measureRectVector"<<endl;
	}
	if(!predictRectVector.empty() )
	{
		predictRectVector.clear();
		cout << "clear predictRectVector"<<endl;
	}

	KF.statePost.at<float>(0) = x;
    KF.statePost.at<float>(1) = y;
    KF.statePost.at<float>(2) = width;
    KF.statePost.at<float>(3) = height;
	KF.statePost.at<float>(4) = 0;
    KF.statePost.at<float>(5) = 0;
    KF.statePost.at<float>(6) = 0;
    KF.statePost.at<float>(7) = 0;
    KF.transitionMatrix = *(Mat_<float>(8, 8) << 1,0,0,0,1,0,0,0,			//initialize post state of kalman filter at random
												 0,1,0,0,0,1,0,0,			//state(x,y,width,height,detaX,detaY,detawidth,detaheight), the bigger value of detaX and detaY, 
												 0,0,1,0,0,0,1,0,			//the faster tracking speed,original 1,1
												 0,0,0,1,0,0,0,1,
												 0,0,0,0,1,0,0,0,
												 0,0,0,0,0,1,0,0,
												 0,0,0,0,0,0,1,0,
												 0,0,0,0,0,0,0,1);

    //setIdentity(KF.measurementMatrix);
    //setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
    //setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    //setIdentity(KF.errorCovPost, Scalar::all(.1));
		
																	//measurement matrix (H) 观测模型
	setIdentity(KF.measurementMatrix);								//setIdentity: initializes a scaled identity matrix; 缩放的单位对角矩阵;
																	//process noise covariance matrix (Q)	
	setIdentity(KF.processNoiseCov, Scalar::all(1e-5));				// wk 是过程噪声，并假定其符合均值为零，协方差矩阵为Qk(Q)的多元正态分布;		
																	//measurement noise covariance matrix (R)
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));			//vk 是观测噪声，其均值为零，协方差矩阵为Rk,且服从正态分布;		
																	//priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/  A代表F: transitionMatrix	
	setIdentity(KF.errorCovPost, Scalar::all(1));					//预测估计协方差矩阵;
																	//corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))

	initRect = Rect(x, y, width, height);
	predictRectVector.push_back(initRect);
    init = true;
	counter = 0;
}

void KalmanFilterTracker::updateKF(int x, int y, int width, int height)
{
	cout<<"KalmanFilterTrackerUpdateKF"<<endl;
    if ( !init )
        initKF(x, y, width, height);

	//step 1: predit next position based on former position
    Mat predictionb = KF.predict();								//prediction before update
    Rect predictRect(predictionb.at<float>(0), predictionb.at<float>(1),predictionb.at<float>(2),predictionb.at<float>(3));
	
	//step 2: update measurement 
	measurement.at<float>(0) = x;
	measurement.at<float>(1) = y;
	measurement.at<float>(2) = width;
	measurement.at<float>(3) = height;

    Rect measureRect(measurement.at<float>(0), measurement.at<float>(1),measurement.at<float>(2),measurement.at<float>(3));
	measureRectVector.push_back(measureRect);					//update measurement
	if(measureRectVector.size() > 20)							//keep 10 point to draw line
		measureRectVector.erase(measureRectVector.begin());		//delete first element

	//step 3: give corrected position based on updated measurement and prediction
    Mat correct = KF.correct(measurement);						//update statepost
    Rect stateRect( correct.at<float>(0), correct.at<float>(1), correct.at<float>(2), correct.at<float>(3) );
    
	predictRectVector.push_back(stateRect);						//keep 10 point to draw line
	if(predictRectVector.size() > 20)
		predictRectVector.erase(predictRectVector.begin());		//delete first element

	
}

void KalmanFilterTracker::updateWithoutMeasurement()
{
	cout<<"KalmanFilterTrackerUpdateWithoutMeasurement"<<endl;
	//step 1: predit next position based on former position
    Mat predictionb = KF.predict();								//prediction before update
    Rect predictRect(predictionb.at<float>(0), predictionb.at<float>(1),predictionb.at<float>(2),predictionb.at<float>(3));
	
	//step 2: update measurement 
	measurement.at<float>(0) = predictionb.at<float>(0);
	measurement.at<float>(1) = predictionb.at<float>(1);
	measurement.at<float>(2) = predictionb.at<float>(2);
	measurement.at<float>(3) = predictionb.at<float>(3);

    Rect measureRect(measurement.at<float>(0), measurement.at<float>(1),measurement.at<float>(2),measurement.at<float>(3));
	measureRectVector.push_back(measureRect);					//update measurement
	if(measureRectVector.size() > 20)							//keep 10 point to draw line
		measureRectVector.erase(measureRectVector.begin());		//delete first element

	//step 3: give corrected position based on updated measurement and prediction
    Mat correct = KF.correct(measurement);						//update statepost
    Rect stateRect( correct.at<float>(0), correct.at<float>(1), correct.at<float>(2), correct.at<float>(3) );
    
	predictRectVector.push_back(stateRect);						//keep 10 point to draw line
	if(predictRectVector.size() > 20)
		predictRectVector.erase(predictRectVector.begin());		//delete first element



}
void KalmanFilterTracker::changeStatus(Mat& img)
{
	size_t i,j;
	warningStatus = ("");
	dangerStatus = ("");
	space = (" + ");
	if( predictRectVector.size() > 0 && predictRectVector.size() < 3)
	{
		warningStatus = string("Tracking Initialized");
	}
	else if( predictRectVector.size() > 3 )
	{
		j = predictRectVector.size() - 1;
		i = predictRectVector.size()/2;
		if( abs((predictRectVector[j].x+(predictRectVector[j].width/2)) - (img.cols/2)) < 
			abs((predictRectVector[i].x+(predictRectVector[i].width/2)) - (img.cols/2)) )
		{
			warningStatus = string("Warning");
			if( (predictRectVector[j].y+predictRectVector[j].height) > (img.rows/2) )
			{
				dangerStatus = string("DANGER!");
			}
		}
	}

}
void KalmanFilterTracker::putStatusText(Mat& img)
{
	size_t j;
	if(predictRectVector.size() > 0)
	{
		j = predictRectVector.size() - 1;
		putText(img, warningStatus, Point( predictRectVector[j].br().x, predictRectVector[j].br().y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1, 8, false);//rbg,255,255,0
		putText(img, dangerStatus, Point( predictRectVector[j].x, predictRectVector[j].y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8, false);//scalar:bgr
	}
}
void KalmanFilterTracker::drawPredictLine(Mat& img)
{
	size_t j = predictRectVector.size() - 1;
	for (size_t i=0; i<predictRectVector.size()-1; ++i)
	{
		line(img, Point((predictRectVector.at(i).x + predictRectVector.at(i).width/2),(predictRectVector.at(i).y + predictRectVector.at(i).height)), 
			Point((predictRectVector.at(i+1).x + predictRectVector.at(i+1).width/2),(predictRectVector.at(i+1).y + predictRectVector.at(i+1).height)), Scalar(0,255,0), 2, 8);//green line
	
	}
	rectangle(img, predictRectVector.at(j).tl(), predictRectVector.at(j).br(),Scalar(0,255,0), 2, 8);//green rectangle
	cout<<"predictRectVector.size() : "<< j <<endl;
}

void KalmanFilterTracker::drawMeasureLine(Mat& img)
{
	size_t j = measureRectVector.size() - 1;
	for (size_t i=0; i<measureRectVector.size()-1; ++i)
	{
		line(img, Point((measureRectVector.at(i).x + measureRectVector.at(i).width/2),(measureRectVector.at(i).y + measureRectVector.at(i).height)), 
			Point((measureRectVector.at(i+1).x + measureRectVector.at(i+1).width/2),(measureRectVector.at(i+1).y + measureRectVector.at(i+1).height)), Scalar(255,0,0), 2, 8);
	}
	rectangle(img, measureRectVector.at(j).tl(), measureRectVector.at(j).br(),Scalar(255,0,0), 2, 8);//blue rectangle
	cout<<"measureRectVector.size() : "<< j <<endl;
}

void KalmanFilterTracker::addcounter()
{
	counter += 1;
}

int KalmanFilterTracker::getcounter()
{
	return counter;
}

void KalmanFilterTracker::drawPretictLinelns(Mat img, double ratio, int resx, int resy, int tempx, int tempy)
{
}

void KalmanFilterTracker::drawMeasureLinelns(Mat img, double ratio, int resx, int resy, int tempx, int tempy)
{
}

