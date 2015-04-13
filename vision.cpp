/*
 * BALINESOFT PRESENTS:
 *
 * "Laika come home"
 *                           ;\
                            |' \
         _                  ; : ;
        / `-.              /: : |
       |  ,-.`-.          ,': : |
       \  :  `. `.       ,'-. : |
        \ ;    ;  `-.__,'    `-.|
         \ ;   ;  :::  ,::'`:.  `.
          \ `-. :  `    :.    `.  \
           \   \    ,   ;   ,:    (\
            \   :., :.    ,'o)): ` `-.
           ,/,' ;' ,::"'`.`---'   `.  `-._
         ,/  :  ; '"      `;'          ,--`.
        ;/   :; ;             ,:'     (   ,:)
          ,.,:.    ; ,:.,  ,-._ `.     \""'/
          '::'     `:'`  ,'(  \`._____.-'"'
             ;,   ;  `.  `. `._`-.  \\
             ;:.  ;:       `-._`-.\  \`.
              '`:. :        |' `. `\  ) \
                 ` ;:       |    `--\__,'
                   '`      ,'
                        ,-'
                        *
 * "A brief story of drones and autonomous landing"
 *
 * Basic structure of the program:
 *
 * 1. Start video stream
 * 2. Change colorspace to HSV
 * 3. Apply color filter
 * 4. Select ROI (RotatedRec and Bounding Rectangle)
 * 5. Find center circle area
 * 6. Get distance from area
 * 7. Center drone
 * 8. Send control command!
 *
 * */

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "vision.h"

using namespace cv;
using namespace std;

//Function headers
void check_Status(int);
int read_Video(VideoCapture cap);
int RGB2HSV();
int do_Threshold(int, int, int, int, int, int);
int do_Morph();
void thresh_callback(int, void*);

//Sort function
    //Helper function to sort the areas in descending order
bool sortByArea(const Moments &lhs, const Moments &rhs)
{
    return lhs.m00 > rhs.m00;
}

//Variables
int status;
bool SHOW = true; //'true' = image output, 'false' = NO image output
//Contour variables
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
unsigned char MaxCont; //Max contour
unsigned int Dist; //Distance of the object
unsigned int RealAr = 3278;
int thresh = 0;
int max_thresh = 255;
RNG rng(12345);
//Mat
Mat imgOriginal;
Mat imgThresholded;
Mat imgHSV;
Mat canny_output;
//VideoCapture cap;

int vision_main_thread()//( int argc, char** argv )
{
    //HSV filter color values
    //Test values! (Yellow-ish)
    int iLowH = 13;
    int iHighH = 82;

    int iLowS = 51;
    int iHighS = 168;

    int iLowV = 159;
    int iHighV = 237;

    //status = start_Video(1);

    VideoCapture cap(1); //capture the video from web cam

    if ( !cap.isOpened() )  // if not success, exit program
    {
        cout << "Cannot open the web cam" << endl;
        status = 1;
    }
    else
    {
        status = 0;
    }

    check_Status(status);

    if (SHOW)
    {
        //Control window
        namedWindow("Control", CV_WINDOW_AUTOSIZE);
        //Create trackbars in "Control" window
        //Hue: Shade of color
        cvCreateTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 179)
        cvCreateTrackbar("HighH", "Control", &iHighH, 255);
        //Saturation: Amount of light/white
        cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
        cvCreateTrackbar("HighS", "Control", &iHighS, 255);
        //Value: Amount of black
        cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
        cvCreateTrackbar("HighV", "Control", &iHighV, 255);

        //Create trackbar for Canny/Contours
        //cvCreateTrackbar("Canny thresh:", "Control", &thresh, max_thresh, thresh_callback);
        createTrackbar("Canny thresh:", "Control", &thresh, max_thresh, thresh_callback);
    }

    while (true)
    {
        //Read new frame
        status = read_Video(cap);
        check_Status(status);

        //PREPROCESSING
        //Blur
        blur(imgHSV, imgHSV, Size(3,3));
        //RGB to HSV
        status = RGB2HSV();
        check_Status(status);

        //PROCESSING
        //Threshold the image
        status = do_Threshold(iLowH, iHighH, iLowS, iHighS, iLowV, iHighV);
        check_Status(status);

        status = do_Morph();
        check_Status(status);

        //MUTE
        if (SHOW)
        {
            imshow("Thresholded Image", imgThresholded); //show the thresholded image
            imshow("Original", imgOriginal); //show the original image
        }

        //Identification
        thresh_callback(0,0);

        //CHECK!
        /*
        Moments oMoments = moments(contours[MaxCont]);
        MaxCont=0; //Restart the value of max count in case the objet area has change

        printf("IN MOMENTS\n");
        double dM01 = oMoments.m01;
        double dM10 = oMoments.m10;
        double dArea = oMoments.m00;
        printf("OUT MOMENTS\n");

        // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
        if (dArea > 100 && dArea < 250000)
        {
            printf("dArea: %f ", dArea);
            //calculate the position of the ball
            unsigned int posX = dM10 / dArea;
            printf(" PosX: %d", posX);
            unsigned int posY = dM01 / dArea;
            printf(" Posy: %d\n", posY);

            //if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
            //{
                //Calculate de distance of an object with a given Area
                Dist=sqrt((RealAr*37)/dArea);
                printf("Distance to object is: %d (m)\n", Dist);
            //}
        }
        //*/
        //Finish CHECK!

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }
    }

           return 0;

}

void check_Status(int status)
{
    if(status != 0)
    {
        //Something is not right
        printf("ERROR!\n");
        //Exit
        exit(EXIT_FAILURE);
    }
    else
    {
        //Free for now
    }
}

int read_Video(VideoCapture cap)
{
    //Mat imgOriginal;

    bool bSuccess = cap.read(imgOriginal); // read a new frame from video

    if (!bSuccess) //if not success, break loop
    {
        cout << "Cannot read a frame from video stream" << endl;
        return 1;
    }
    else
    {
        //We're good!
        return 0;
    }
}

int RGB2HSV()
{
    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    //We hope we're good
    return 0;
}

int do_Threshold(int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV)
{
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
    //We hope we're good
    return 0;
}

int do_Morph()
{
    //Check for sizes and Structuring Elements
    //morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    //3x3
    //morphological closing (fill small holes in the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    //eliminar
    //We should be good!
    return 0;
}

void thresh_callback(int, void* )
{
    float Xmin=0;
    float Xmax=0;
    float Ymin=0;
    float Ymax=0;
    Mat canny_output;
    //GLOBAL NOW!
    //vector<vector<Point> > contours;
    //vector<Vec4i> hierarchy;

    //Detect edges using canny
    //Canny( imgThresholded, canny_output, thresh, thresh*3, 3 );
        //imgThresholded -> image
        //canny_output -> output
        //tresh ->
        //tresh*3 -> Recommended value
        //3 -> Kernel size (recommended value)
    Canny(imgThresholded, canny_output, thresh, thresh*3, 3);

    //Find contours
    findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

    //Get the moments
    vector<Moments> mu(contours.size() );
    for( size_t i = 0; i < contours.size(); i++ )
    {
        mu[i] = moments( contours[i], false );
        printf("Got contour %i\n", i);
    }

    //TEMP CHECK!
    //Print original info
    for( size_t i = 0; i< contours.size(); i++ )
    {
        printf("OG: * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", (int)i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
    }
    //Sort vector
    sort(mu.begin(), mu.end(), sortByArea);

    //Print sorted vector
    for( size_t i = 0; i< contours.size(); i++ )
    {
        printf("SORT: * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", (int)i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
    }
    //Get the mass centers:
    vector<Point2f> mc( contours.size() );
    for( size_t i = 0; i < contours.size(); i++ )
    //Get only 4
    //for( size_t i = 0; i < 4; i++ )
    {
        mc[i] = Point2f( static_cast<float>(mu[i].m10/mu[i].m00) , static_cast<float>(mu[i].m01/mu[i].m00) );
        printf("For contour %i: X=%f, Y=%f\n", i, mc[i].x, mc[i].y);
    }

    //Draw contours
    if (SHOW)
    {
        Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
        for( size_t i = 0; i< contours.size(); i++ )
        {
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, Point() );
            circle( drawing, mc[i], 4, color, -1, 8, 0 );
        }
        namedWindow( "Contours", WINDOW_AUTOSIZE );
        imshow( "Contours", drawing );
    }
    //Get coordinates for ROI
        //If for some reason there is no previous ROI, check if there is an error
    if(contours.size()>0)
    {
        if(contours.size()<4)
        {
            //ERROR!
            //Send message
            Xmin=0;
            Xmax=(float)imgOriginal.cols;
            Ymin=0;
            Ymax=(float)imgOriginal.rows;
        }
        else
        {
            int j;
            //Xmin
            Xmin=mc[0].x;
            for (j = 0; j < 4; j++)
            {
                if (Xmin > mc[j].x)
                {
                    Xmin = mc[j].x;
                }
            }
            //Xmax
            Xmax=mc[0].x;
            for (j = 0; j < 4; j++)
            {
                if (Xmax < mc[j].x)
                {
                    Xmax = mc[j].x;
                }
            }
            //Ymin
            Ymin=mc[0].y;
            for (j = 0; j < 4; j++)
            {
                if (Ymin > mc[j].y)
                {
                    Ymin = mc[j].y;
                }
            }
            //Ymax
            Ymax=mc[0].y;
            for (j = 0; j < 4; j++)
            {
                if (Ymax < mc[j].y)
                {
                    Ymax = mc[j].y;
                }
            }
        }
    }
    else
    {
        //Nothing found!
        //Send message
        Xmin=0;
        Xmax=(float)imgOriginal.cols;
        Ymin=0;
        Ymax=(float)imgOriginal.rows;
    }

    //Print ROI (x,y)
    printf("Got Xmin = %f Xmax = %f Ymin = %f Ymax = %f\n", Xmin, Xmax, Ymin, Ymax);

    //Check if X and Y are the same
    if (Xmin == Xmax || Ymin == Ymax)
    {
        //WUT
        //Send message
        Xmin=0;
        Xmax=(float)imgOriginal.cols;
        Ymin=0;
        Ymax=(float)imgOriginal.rows;
    }

    //ROI in original image
    //Rect (Xmin, Ymin, Width, Height)
    Mat ROI = imgOriginal( Rect (Xmin, Ymin, Xmax-Xmin, Ymax-Ymin) );
    imshow("ROI", ROI);
}

int spec_Canny()
{
	//Sigma
	int sigma=0.33;

	float lowCann;
	float upCann;

	//Get median


}
