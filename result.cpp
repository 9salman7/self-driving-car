#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <wiringPi.h>

using namespace std;
using namespace cv;
using namespace raspicam;

Mat frame, Matrix, framePers, frameGray, frameThresh, frameEdge, frameFinal, frameFinalDuplicate;
Mat ROILane;
int LeftLanePos, RightLanePos, frameCenter, laneCenter, Result;

RaspiCam_Cv Camera;

stringstream ss;


vector<int> histrogramLane;   //dynamic array for histogram

Point2f Source[] = {Point2f(40,145),Point2f(360,145),Point2f(10,195), Point2f(390,195)};    //points for creating region of interest
Point2f Destination[] = {Point2f(100,0),Point2f(280,0),Point2f(100,240), Point2f(280,240)};  //points for perspective view


 void Setup ( int argc,char **argv, RaspiCam_Cv &Camera )
  {
    Camera.set ( CAP_PROP_FRAME_WIDTH,  ( "-w",argc,argv,400 ) );
    Camera.set ( CAP_PROP_FRAME_HEIGHT,  ( "-h",argc,argv,240 ) );
    Camera.set ( CAP_PROP_BRIGHTNESS, ( "-br",argc,argv,50 ) );
    Camera.set ( CAP_PROP_CONTRAST ,( "-co",argc,argv,50 ) );
    Camera.set ( CAP_PROP_SATURATION,  ( "-sa",argc,argv,50 ) );
    Camera.set ( CAP_PROP_GAIN,  ( "-g",argc,argv ,50 ) );
    Camera.set ( CAP_PROP_FPS,  ( "-fps",argc,argv,100));

}

void Capture()   
{
	Camera.grab();
    Camera.retrieve( frame);
    cvtColor(frame, frame, COLOR_BGR2RGB);  //convert captured image to rgb colour space
}

void Perspective()  //get bird's eye view of image
{
	line(frame,Source[0], Source[1], Scalar(0,0,255), 2);    //create line from 1st point to second point, third argument defines colour of line, fourth is width
	line(frame,Source[1], Source[3], Scalar(0,0,255), 2);
	line(frame,Source[3], Source[2], Scalar(0,0,255), 2);
	line(frame,Source[2], Source[0], Scalar(0,0,255), 2);
	
	
	Matrix = getPerspectiveTransform(Source, Destination);   
	warpPerspective(frame, framePers, Matrix, Size(400,240));  //perspective transformation of the original image- framePers is the result
}

void Threshold()  //create threshold for whites and blacks
{
	cvtColor(framePers, frameGray, COLOR_RGB2GRAY);   //convert perspective image to grayscale
	inRange(frameGray, 200, 255, frameThresh);    //second parameter is min threshold (adjust this accordingly), third is max
	Canny(frameGray,frameEdge, 900, 900, 3, false);  //edge detection- adjust accordingly
	add(frameThresh, frameEdge, frameFinal);   //merge threshold image and edge detection image
	cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB);  
	cvtColor(frameFinal, frameFinalDuplicate, COLOR_RGB2BGR);   //used in histrogram function only
	
}

void Histrogram()
{
    histrogramLane.resize(400);  //resize to 400 width
    histrogramLane.clear();
    
    for(int i=0; i<400; i++)       //frame.size().width = 400
    {
	ROILane = frameFinalDuplicate(Rect(i,140,1,100));  //divide into 400 strips of 1px width
	divide(255, ROILane, ROILane);  //normalize
	histrogramLane.push_back((int)(sum(ROILane)[0]));  //push into the array 
    }
}

void LaneFinder()   //to find the positions of lanes from the vector
{
    vector<int>:: iterator LeftPtr;
    LeftPtr = max_element(histrogramLane.begin(), histrogramLane.begin() + 150);  //start and end of array
    LeftLanePos = distance(histrogramLane.begin(), LeftPtr);  //gets the position (distance from left side of frame) of left lane
    
    vector<int>:: iterator RightPtr;
    RightPtr = max_element(histrogramLane.begin() +250, histrogramLane.end());
    RightLanePos = distance(histrogramLane.begin(), RightPtr);
    
    line(frameFinal, Point2f(LeftLanePos, 0), Point2f(LeftLanePos, 240), Scalar(0, 255,0), 2);  //draw lines for lanes
    line(frameFinal, Point2f(RightLanePos, 0), Point2f(RightLanePos, 240), Scalar(0,255,0), 2); 
}

void LaneCenter()
{
    laneCenter = (RightLanePos-LeftLanePos)/2 +LeftLanePos;
    frameCenter = 188;  //adjust this (keep car in exact center)
    
    line(frameFinal, Point2f(laneCenter,0), Point2f(laneCenter,240), Scalar(0,255,0), 3);
    line(frameFinal, Point2f(frameCenter,0), Point2f(frameCenter,240), Scalar(255,0,0), 3);

    Result = laneCenter-frameCenter;  
}


int main(int argc,char **argv)
{
	wiringPiSetup();
    pinMode(21, OUTPUT);
    pinMode(22, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(24, OUTPUT);
	

	Setup(argc, argv, Camera);
	cout<<"Connecting to camera"<<endl;
	if (!Camera.open())
	{	
		cout<<"Failed to Connect"<<endl;
    } 
    cout<<"Camera Id = "<<Camera.getId()<<endl;
     
    while(1)
    {
		auto start = std::chrono::system_clock::now();

	    Capture();
	    Perspective();
	    Threshold();
	    Histrogram();
	    LaneFinder();
	    LaneCenter();
	    
	    ss.str(" ");
	    ss.clear();
	    ss<<"Result = "<<Result;
	    putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
	    
	    if (Result == 0)
	    {
			digitalWrite(21, 0);
			digitalWrite(22, 0);    //decimal = 0
			digitalWrite(23, 0);
			digitalWrite(24, 0);
			cout<<"Forward"<<endl;
	    }
	        
	    else if (Result >0 && Result <10)
	    {
			digitalWrite(21, 1);
			digitalWrite(22, 0);    //decimal = 1
			digitalWrite(23, 0);
			digitalWrite(24, 0);
			cout<<"Right1"<<endl;
	    }
	    
	    else if (Result >=10 && Result <20)
	    {
			digitalWrite(21, 0);
			digitalWrite(22, 1);    //decimal = 2
			digitalWrite(23, 0);
			digitalWrite(24, 0);
			cout<<"Right2"<<endl;
	    }
	    
	    else if (Result >20)
	    {
			digitalWrite(21, 1);
			digitalWrite(22, 1);    //decimal = 3
			digitalWrite(23, 0);
			digitalWrite(24, 0);
			cout<<"Right3"<<endl;
	    }
	    
        else if (Result <0 && Result >-10)
	    {
			digitalWrite(21, 0);
			digitalWrite(22, 0);    //decimal = 4
			digitalWrite(23, 1);
			digitalWrite(24, 0);
			cout<<"Left1"<<endl;
	    }
	    
        else if (Result <=-10 && Result >-20)
	    {
			digitalWrite(21, 1);
			digitalWrite(22, 0);    //decimal = 5
			digitalWrite(23, 1);
			digitalWrite(24, 0);
			cout<<"Left2"<<endl;
	    }
	    
	    else if (Result <-20)
	    {
			digitalWrite(21, 0);
			digitalWrite(22, 1);    //decimal = 6
			digitalWrite(23, 1);
			digitalWrite(24, 0);
			cout<<"Left3"<<endl;
	    }
	    
	    namedWindow("Original Frame", WINDOW_KEEPRATIO);
	    moveWindow("Original Frame", 0, 100);
	    resizeWindow("Original Frame", 640, 480);
	    imshow("Original Frame", frame);
	    
	    namedWindow("Perspective", WINDOW_KEEPRATIO);
	    moveWindow("Perspective", 640, 100);
	    resizeWindow("Perspective", 640, 480);
	    imshow("Perspective", framePers);
	    
	    namedWindow("Final", WINDOW_KEEPRATIO);
	    moveWindow("Final", 1280, 100);
	    resizeWindow("Final", 640, 480);
	    imshow("Final", frameFinal);
	    
	    waitKey(1);
	    auto end = std::chrono::system_clock::now();
	    std::chrono::duration<double> elapsed_seconds = end-start;
	    
	    float t = elapsed_seconds.count();
	    int FPS = 1/t;
	    cout<<"FPS = "<<FPS<<endl;
    }
    return 0;
}
