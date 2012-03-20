//============================================================================
// Name        : KinectTouch.cpp
// Author      : github.com/robbeofficial
// Version     : 0.something
// Description : recognizes touch points on arbitrary surfaces using kinect
// 				 and maps them to TUIO cursors
// 				 (turns any surface into a touchpad)
//============================================================================

/*
 * 1. point your kinect from a higher place down to your table
 * 2. start the program (keep your hands off the table for the beginning)
 * 3. use your table as a giant touchpad
 */
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <map>
using namespace std;

// openCV
#include <opencv/highgui.h>
#include <opencv/cv.h>
using namespace cv;

// openNI
#include <XnOpenNI.h>
#include <XnCppWrapper.h>
using namespace xn;
#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;													\
	}

// TUIO
#include "TuioServer.h"
using namespace TUIO;

#include <tinyxml.h>
#include <tinystr.h>

// TODO smoothing using kalman filter

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------

// OpenNI
xn::Context xnContext;
xn::DepthGenerator xnDepthGenerator;
xn::ImageGenerator xnImgeGenertor;

// Mouse / 4 points calibration

//bool mousePressed = false;
Point cursor;
int calibration_point_index=0;  // index of current calibration point being changed
Point calibration_points[4];
double xy_ratio=1.0;
bool setting_calibration_points=false;
Mat H; // perspective_transform matrix

// ROI boundaries
int xMin = 110;
int xMax = 560;
int yMin = 120;
int yMax = 320;

//TUIO
std::string kt_host="localhost";
int kt_port=3333;

//blob detection
const unsigned int nBackgroundTrain = 30;
	// unsigned short
int touchDepthMin = 10;
	 //unsigned short
int touchDepthMax = 20;
	 //unsigned
int touchMinArea = 50;
int touchMaxArea = 150;

// delay between 2 frames
int Waitdelay = 100;

const char* windowName = "Debug";
const char* controlName = "Controls";

//---------------------------------------------------------------------------
// Functions
//---------------------------------------------------------------------------

int initOpenNI(const XnChar* fname) {
	XnStatus nRetVal = XN_STATUS_OK;

	// initialize context
	nRetVal = xnContext.InitFromXmlFile(fname);
	CHECK_RC(nRetVal, "InitFromXmlFile");

	// initialize depth generator
	nRetVal = xnContext.FindExistingNode(XN_NODE_TYPE_DEPTH, xnDepthGenerator);
	CHECK_RC(nRetVal, "FindExistingNode(XN_NODE_TYPE_DEPTH)");

	// initialize image generator
	//nRetVal = xnContext.FindExistingNode(XN_NODE_TYPE_IMAGE, xnImgeGenertor);
	//CHECK_RC(nRetVal, "FindExistingNode(XN_NODE_TYPE_IMAGE)");

	return 0;
}






Mat find_perspective_transform()
{
    vector<Point2f>pointsIn;
    vector<Point2f>pointsRes;

    for(int i=0;i<4;i++)
    pointsIn.push_back(calibration_points[i]);

    pointsRes.push_back(Point(0,0));
    pointsRes.push_back(Point(1,0));
    pointsRes.push_back(Point(1,1));
    pointsRes.push_back(Point(0,1));

    Mat m =getPerspectiveTransform(pointsIn,pointsRes);
return(m);
}

void onMouse( int event, int x, int y, int, void* )
{
    vector<Point2f>  mouse_point(1);
    vector<Point2f>  scene_point;
    mouse_point[0]=Point(x,y);

    if( event != CV_EVENT_LBUTTONDOWN )
       return;
    if (setting_calibration_points)
    {
        calibration_points[calibration_point_index] = Point(x,y);
        printf ("calibration point n°%d\n",calibration_point_index);
        calibration_point_index+=1;
        if (calibration_point_index==4)
        {
            // updating ROI boundaries
            xMin=min(min(calibration_points[0].x,calibration_points[1].x),min(calibration_points[2].x,calibration_points[3].x));
            xMax=max(max(calibration_points[0].x,calibration_points[1].x),max(calibration_points[2].x,calibration_points[3].x));
            yMin=min(min(calibration_points[0].y,calibration_points[1].y),min(calibration_points[2].y,calibration_points[3].y));
            yMax=max(max(calibration_points[0].y,calibration_points[1].y),max(calibration_points[2].y,calibration_points[3].y));

            // updating trackbars
            setTrackbarPos(	"xMin", controlName , xMin);
            setTrackbarPos(	"xMax", controlName , xMax);
            setTrackbarPos(	"yMin", controlName , yMin);
            setTrackbarPos(	"yMax", controlName , yMax);

            H=find_perspective_transform();
            setting_calibration_points=false;
            printf("Matrice de transformation calculee...\n");
            cout<<H;
        }
    }
    else
    {
        if(calibration_point_index==4)
        {
            perspectiveTransform(mouse_point,scene_point,H);
            printf ("mouse_new_coords=%f %f\n",scene_point[0].x,scene_point[0].y);
        }
    }

    return;
}
/// ////////



void average(vector<Mat1s>& frames, Mat1s& mean) {
	Mat1d acc(mean.size());
	Mat1d frame(mean.size());

	for (unsigned int i=0; i<frames.size(); i++) {
		frames[i].convertTo(frame, CV_64FC1);
		acc = acc + frame;
	}

	acc = acc / frames.size();

	acc.convertTo(mean, CV_16SC1);
}

//  xml settings

/*Function to load parameters*/
std::string loadParameters()
{
    printf("loading....\n");
    int read=0;
    TiXmlDocument doc( "settings.xml" );
    bool loadOkay=doc.LoadFile();
    if(loadOkay)
    {
        TiXmlElement * root=doc.FirstChildElement();
        TiXmlElement * valores=root->FirstChildElement();
        TiXmlElement * blobDet=valores->FirstChildElement("Blob-Detection");
      //  TiXmlElement * affineVal=valores->FirstChildElement("Affine-Calibration");
        TiXmlElement * tuioInfo=valores->FirstChildElement("Tuio-Info");
        TiXmlElement * pointCal=valores->FirstChildElement("Point-Calibration");
        read=blobDet->QueryIntAttribute("Min-Distance",&touchDepthMin);
        read=blobDet->QueryIntAttribute("Max-Distance",&touchDepthMax);
        read=blobDet->QueryIntAttribute("Min-Blob-Area",&touchMinArea);
        read=blobDet->QueryIntAttribute("Max-Blob-Area",&touchMaxArea);
        read=blobDet->QueryIntAttribute("Delay",&Waitdelay);

        read=pointCal->QueryIntAttribute("x0",&calibration_points[0].x);
        read=pointCal->QueryIntAttribute("y0",&calibration_points[0].y);
        read=pointCal->QueryIntAttribute("x1",&calibration_points[1].x);
        read=pointCal->QueryIntAttribute("y1",&calibration_points[1].y);
        read=pointCal->QueryIntAttribute("x2",&calibration_points[2].x);
        read=pointCal->QueryIntAttribute("y2",&calibration_points[2].y);
        read=pointCal->QueryIntAttribute("x3",&calibration_points[3].x);
        read=pointCal->QueryIntAttribute("y3",&calibration_points[3].y);
        read=pointCal->QueryDoubleAttribute("xy_ratio",&xy_ratio);

		read=tuioInfo->QueryIntAttribute("Port",&kt_port);

        cout << "Parameter file: settings.xml - Loaded!" << endl;

          // updating ROI boundaries
            xMin=min(min(calibration_points[0].x,calibration_points[1].x),min(calibration_points[2].x,calibration_points[3].x));
            xMax=max(max(calibration_points[0].x,calibration_points[1].x),max(calibration_points[2].x,calibration_points[3].x));
            yMin=min(min(calibration_points[0].y,calibration_points[1].y),min(calibration_points[2].y,calibration_points[3].y));
            yMax=max(max(calibration_points[0].y,calibration_points[1].y),max(calibration_points[2].y,calibration_points[3].y));

            // updating trackbars
            setTrackbarPos(	"xMin", controlName , xMin);
            setTrackbarPos(	"xMax", controlName , xMax);
            setTrackbarPos(	"yMin", controlName , yMin);
            setTrackbarPos(	"yMax", controlName , yMax);
            setTrackbarPos("DepthMin", controlName, touchDepthMin);
            setTrackbarPos("DepthMax", controlName, touchDepthMax);
            setTrackbarPos("MinArea", controlName, touchMinArea);
            setTrackbarPos("MaxArea", controlName, touchMaxArea);
            setTrackbarPos("Delay", controlName, Waitdelay);
            // computing transform matrix
            H=find_perspective_transform();
            calibration_point_index=4;  // trick for enabling perpective transfom mode
            printf("Matrice de transformation calculee...\n");

		return tuioInfo->Attribute("Host");
   }
    else
    {
        cout << "Parameter file: settings.xml - not found. Using default values." << endl;
    }
	return "localhost";
}
/*FUNCTION TO SAVE PARAMETERS*/
void saveParameters()//int a,void* param)
{
    printf("saving....\n");
    TiXmlDocument doc;
    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
    doc.LinkEndChild( decl );
    TiXmlElement * root = new TiXmlElement( "Parameters" );
    TiXmlElement * valores = new TiXmlElement( "Values" );
    doc.LinkEndChild( root );
    root->LinkEndChild( valores );
    TiXmlElement * blobDet= new TiXmlElement( "Blob-Detection" );
    TiXmlElement * tuioInfo= new TiXmlElement( "Tuio-Info" );
    TiXmlElement * pointCal= new TiXmlElement( "Point-Calibration" );
    valores->LinkEndChild( blobDet );
    valores->LinkEndChild( pointCal );
    valores->LinkEndChild( tuioInfo );
    blobDet->SetAttribute("Min-Distance", touchDepthMin);
    blobDet->SetAttribute("Max-Distance", touchDepthMax);
    blobDet->SetAttribute("Min-Blob-Area", touchMinArea);
    blobDet->SetAttribute("Max-Blob-Area",touchMaxArea);
    blobDet->SetAttribute("Delay",Waitdelay);

    pointCal->SetAttribute("x0",calibration_points[0].x);
    pointCal->SetAttribute("y0",calibration_points[0].y);
    pointCal->SetAttribute("x1",calibration_points[1].x);
    pointCal->SetAttribute("y1",calibration_points[1].y);
    pointCal->SetAttribute("x2",calibration_points[2].x);
    pointCal->SetAttribute("y2",calibration_points[2].y);
    pointCal->SetAttribute("x3",calibration_points[3].x);
    pointCal->SetAttribute("y3",calibration_points[3].y);
    pointCal->SetAttribute("xy_ratio",xy_ratio);


	tuioInfo->SetAttribute("Host",kt_host.c_str());//loadParameters().c_str());
    tuioInfo->SetAttribute("Port",kt_port);
     bool saveOk=doc.SaveFile( "settings.xml" );
    if(saveOk)
    {
        cout << "Parameters saved!" << endl;
    }
    else
    {
        cout << "Parameters could not be saved." << endl;
    }
}



int main() {

    printf ("KINECT TOUCH2\n");

    kt_host=loadParameters().c_str();

	 const unsigned int nBackgroundTrain = 30;


	const bool localClientMode = true; 					// connect to a local client

	const double debugFrameMaxDepth = 4000; // maximal distance (in millimeters) for 8 bit debug depth frame quantization
	const Scalar debugColor0(0,0,255);
	const Scalar debugColor1(255,0,0);
	const Scalar debugColor2(255,255,255);
	const Scalar circleColor(0,255,255);

     setting_calibration_points=false;
    bool affiche_debug=true;

    bool xy_ratio_landscape= (xy_ratio>1.0);
    bool xy_ratio_portrait= (xy_ratio<1.0);





	Mat1s depth(480, 640); // 16 bit depth (in millimeters)
	Mat1b depth8(480, 640); // 8 bit depth
	Mat3b rgb(480, 640); // 8 bit depth

	Mat3b debug(480, 640); // debug visualization

	Mat1s foreground(640, 480);
	Mat1b foreground8(640, 480);

	Mat1b touch(640, 480); // touch mask

	Mat1s background(480, 640);
	vector<Mat1s> buffer(nBackgroundTrain);

	printf ("Initialisation Kinect.....");
    int initOpenNIResult=initOpenNI("niConfig.xml");



	// TUIO server object
	TuioServer* tuio;
/*	if (localClientMode) {
		tuio = new TuioServer();
	} else {
		tuio = new TuioServer("192.168.0.2",3333,false);
	}*/
        tuio = new TuioServer(kt_host.c_str(),kt_port,false);
	TuioTime time;

	// create some sliders
	namedWindow(windowName);
    namedWindow(controlName);

    /// active callback souris
    setMouseCallback( windowName, onMouse, 0 );

    createTrackbar("xMin", controlName, &xMin, 635);
	createTrackbar("xMax", controlName, &xMax, 640);
	createTrackbar("yMin", controlName, &yMin, 475);
	createTrackbar("yMax", controlName, &yMax, 480);

    createTrackbar("DepthMin", controlName, &touchDepthMin, 150);
	createTrackbar("DepthMax", controlName, &touchDepthMax, 150);
	createTrackbar("MinArea", controlName, &touchMinArea, 150);
    createTrackbar("MaxArea", controlName, &touchMaxArea, 1000);
    createTrackbar("Delay", controlName, &Waitdelay, 500);
if(initOpenNIResult==0)
	{
	printf ("OK\n");
    printf ("esc : quit\n");
	printf("d : refresh/freeze Display\n");
	printf("b : update Background image\n");
	printf("c : set 4 Calibration points\n");
	printf("p : interpolate background Plane\n");

	// create background model (average depth)
	printf("creating background image...");
	for (unsigned int i=0; i<nBackgroundTrain; i++) {
		xnContext.WaitAndUpdateAll();
		depth.data = (uchar*) xnDepthGenerator.GetDepthMap();
		buffer[i] = depth;
	}
	average(buffer, background);
	printf("OK.\n");
    int code_touche;
	while ( (code_touche=waitKey(max(1,Waitdelay))) != 27 ) {

		/// keyboard options
		if (code_touche=='d') //(enable/disable refresh of debug window)
		{
            affiche_debug=!affiche_debug;
            if (affiche_debug)
                printf("%s window enabled\n", windowName);
            else
                printf("%s window disabled\n", windowName);
            waitKey(10);
		}
		if (code_touche == 'b') // b  (recalibrate)
		{
		    waitKey(500);
		    printf("updating background image...");
		    // create background model (average depth)
            for (unsigned int i=0; i<nBackgroundTrain; i++) {
                xnContext.WaitAndUpdateAll();
                depth.data = (uchar*) xnDepthGenerator.GetDepthMap();
                buffer[i] = depth;
            }
            average(buffer, background);
            printf("OK.\n");
		}
        if (code_touche == 'c')
        { // set calibration points
            setting_calibration_points=true;
            calibration_point_index=0;
            printf("Defining 4 calibration points.... \n(draw a 4 points polygon by clicking 4 times \ninside the %s window with the mouse)\n",windowName);
        }

        if (code_touche == 'p')
        { // compute background plane from calibration points
            if (calibration_point_index==4)  //the 4 points used for calibration are well defined
            {
                printf("computing background plane interpolated from calibration points.... \n",windowName);

            // extracting depthvalue from background image for the 4 calibration points
                short c0=background.at<short>(calibration_points[0].y,calibration_points[0].x);
                short c1=background.at<short>(calibration_points[1].y,calibration_points[1].x);
                short c2=background.at<short>(calibration_points[2].y,calibration_points[2].x);
                short c3=background.at<short>(calibration_points[3].y,calibration_points[3].x);
               // printf("c0=%d c1=%d c2=%d c3=%d\n",c0,c1,c2,c3);
                if ((c0==0)||(c1==0)||(c2==0)||(c3==0))
                {
                printf("ERROR : Depth not defined for all calibartion points\n");
                printf("skipping plane interpolation\n");
                }
                else
                {
                    vector<Point2f>  plane_point(1);
                    vector<Point2f>  result_point;

                    short interpol_depth_value;
                    for(int x=xMin;x<=xMax;x++)
                    {
                        for (int y=yMin;y<=yMax;y++)
                        {
                            plane_point[0]=Point(x,y);
                            perspectiveTransform(plane_point,result_point,H);
                            // compute interpolated depth value
                            interpol_depth_value=saturate_cast<short>((c0*(1.0f-result_point[0].x)+c1*result_point[0].x)*(1.0f-result_point[0].y)+
                                    (c3*(1.0f-result_point[0].x)+c2*result_point[0].x)*result_point[0].y);
                            // copy depthpixel into depthmap
                            background.at<short>(y,x)=interpol_depth_value;
                        }
                    }
                    printf("done.\n");
                }
            }
            else
            printf("you must define 4 calibration points before computing background plane\n");
        }




		// read available data
		xnContext.WaitAndUpdateAll();
		// update 16 bit depth matrix
		depth.data = (uchar*) xnDepthGenerator.GetDepthMap();
		//xnImgeGenertor.GetGrayscale8ImageMap()

		// update rgb image
		//rgb.data = (uchar*) xnImgeGenertor.GetRGB24ImageMap(); // segmentation fault here
		//cvtColor(rgb, rgb, CV_RGB2BGR);

		// extract foreground by simple subtraction of very basic background model
		foreground = background - depth;

		/// find touch mask by thresholding (points that are close to background = touch points)
		touch = (foreground > touchDepthMin) & (foreground < touchDepthMax);

        /// check consistance of (x|y)(Min|Max) values
		if ((xMin+5) > xMax)
            xMax=xMin+5;
        if ((yMin+5) > yMax)
            yMax=yMin+5;

        // extract ROI
        Rect roi(xMin, yMin, xMax - xMin, yMax - yMin);
		Mat touchRoi = touch(roi);

		// find touch points
		vector< vector<Point2i> > contours;
		vector<Point2f> touchPoints;



		findContours(touchRoi, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point2i(xMin, yMin));
		float area;
		for (unsigned int i=0; i<contours.size(); i++) {
			Mat contourMat(contours[i]);
			area=contourArea(contourMat);
			// find touch points by area thresholding
			if ( (area > touchMinArea ) &&((area < touchMaxArea ))){
				Scalar center = mean(contourMat);
				Point2i touchPoint(center[0], center[1]);
				touchPoints.push_back(touchPoint);
			}
		}


		// send TUIO cursors
		time = TuioTime::getSessionTime();
		tuio->initFrame(time);

		for (unsigned int i=0; i<touchPoints.size(); i++) { // touch points
			float cursorX;
			float cursorY;

		if (calibration_point_index!=4)  //we dont use 4points calibration
			{
                cursorX = (touchPoints[i].x - xMin) / (xMax - xMin);
                cursorY = 1 - (touchPoints[i].y - yMin)/(yMax - yMin);
			}
            else
            {
                vector<Point2f>  incomingTouchPoint(1);
                vector<Point2f> warpedTouchPoint(1);
                incomingTouchPoint[0]=touchPoints[i];
                perspectiveTransform(incomingTouchPoint,warpedTouchPoint,H);

                cursorX=warpedTouchPoint[0].x;
                cursorY=warpedTouchPoint[0].y;
                if((cursorX<0)||(cursorX>1)||(cursorY<0)||(cursorY>1))
                continue;

            }

            // adjust xy ratio if necessary
            if(xy_ratio_landscape)
                cursorY=((cursorY-0.5)/xy_ratio)+0.5;
            if(xy_ratio_portrait)
                cursorX=((cursorX-0.5)*xy_ratio)+0.5;


			TuioCursor* cursor = tuio->getClosestTuioCursor(cursorX,cursorY);
			// TODO improve tracking (don't move cursors away, that might be closer to another touch point)
			if (cursor == NULL || cursor->getTuioTime() == time) {
				tuio->addTuioCursor(cursorX,cursorY);
			} else {
				tuio->updateTuioCursor(cursor, cursorX, cursorY);
			}
		}

		tuio->stopUntouchedMovingCursors();
		tuio->removeUntouchedStoppedCursors();
		tuio->commitFrame();
    if (affiche_debug)
    {
		// draw debug frame
		depth.convertTo(depth8, CV_8U, 255 / debugFrameMaxDepth); // render depth to debug frame
		//background.convertTo(depth8, CV_8U, 255 / debugFrameMaxDepth); // render depth to debug frame
		cvtColor(depth8, debug, CV_GRAY2BGR);
		debug.setTo(debugColor0, touch);  // touch mask
		rectangle(debug, roi, debugColor1, 2); // surface boundaries
		for (unsigned int i=0; i<touchPoints.size(); i++) { // touch points
			circle(debug, touchPoints[i], 5, debugColor2, CV_FILLED);
		}

      //   /// affichage points de calibration
         for (int i=0;i<3;i++)
         {
             line(debug, calibration_points[i], calibration_points[i+1], circleColor, 1);
            circle(debug, calibration_points[i], 5, circleColor, CV_FILLED);
         }
            circle(debug, calibration_points[3], 5, circleColor, CV_FILLED);
            line(debug, calibration_points[3], calibration_points[0], circleColor, 1);

		// render debug frame (with sliders)
		imshow(windowName, debug);
		//imshow("image", rgb);
    }
	}
	saveParameters();
	}
	else
	{
	    printf("press esc to exit");
        while ( waitKey(1) != 27 ){}
	}
	//return 0;
}
