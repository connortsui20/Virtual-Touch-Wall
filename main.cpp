/*******************************************************************************************************************************
                                                                 CONNOR TSUI'S TEST CODE
********************************************************************************************************************************/


#include <windows.h> // Windows Header

#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>

#include <vector>
#include <iterator>
#include <cmath>


using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_driver.lib")
#endif


//Defines a 2D point (x, y), also holds a cluster ID
struct Point2D {
    float x, y;
    int cluster; //default cluster set to -1

    Point2D() : x(0), y(0), cluster(-1) {}
    Point2D(float nx, float ny) : x(nx), y(ny), cluster(-1) {}

    float distance(Point2D p) {
        return (p.x - x) * (p.x - x) + (p.y - y) * (p.y - y); //do the square later
    }
}; //Could potentially add a velocities and accleration component to this structure


//Define the size of the screen by showing the distance of the wall away from the sensor
struct ScreenSizeStruct {
    float min_x, max_x, min_y, max_y;
    ScreenSizeStruct(float nx_min, float nx_max, float ny_min, float ny_max) : min_x(nx_min), max_x(nx_max), min_y(ny_min), max_y(ny_max) {}
};


//Changes # of decimal places, only if you want to use this
float round_up(float var) { 
    float value = (int)(var * 1000);
    return (float)value / 1000;
}


//Takes the polar coordinate data from the LiDAR
static void polarOutput(const LaserPoint& point, float& theta, float& r) {
    theta = point.angle + (4*M_PI/180); //angle in radians + adjustment (4 degree hardware adjustment)
    r = point.range; //r in meters
}


//For the LiDAR to only scan down from the top, angles must be between ~pi/2 and pi or ~-pi/2 and -pi
static void limitScreenPolar(bool& inFOV, float& theta, float& r, const ScreenSizeStruct screenSize) {
    float posAdjust, negAdjust;
    posAdjust = atan(screenSize.max_y / screenSize.max_x); //adjust for the distance from the screen to the sensor
    negAdjust = atan(screenSize.max_y / screenSize.min_x);
    if ((theta >= (M_PI/2)+posAdjust && theta <= M_PI) || (theta <= (-M_PI/2)-negAdjust && theta >= -M_PI)) {
        inFOV = true;
    } else { 
        theta = NULL;
        r = NULL;
        inFOV = false; 
    }
}


//Converts the polar coordinates from polarOutput() to Cartesian coordinates
static void polarToCartesian(float theta, float r, float& initX, float& initY) {
        initX = r * sin(theta);
        initY = r * cos(theta);
}


//Limit the x and y data coming out to be within a set range and shift the points so that top left is 0,0
static void cartesianToScreen(float initX, float initY, const ScreenSizeStruct screenSize, float error,  float& x, float& y) {
    if (initX >= screenSize.min_x+error && initX <= screenSize.max_x-error && initY >= screenSize.min_y+error && initY <= screenSize.max_y-error) {
        x = initX + -screenSize.min_x;
        y = -initY + screenSize.max_y; 
    } else { //if the point is not within the bounds of the screen
        x = NULL; //Not sure if this should be NULL, could lead to issues?
        y = NULL;
    }
}


//Explanation in documentation
void touchClustering(vector<Point2D>* detectedPoints, vector<Point2D>& touchPoints, float radius) {

        vector <Point2D> IDPoints; //empty vector to send the ID'd coordinates to
        Point2D startPoint;
        Point2D emptyPoint;
        vector<int> nPoints; // amount of points for average/centroid calculation
        vector<float> sumX, sumY; //sum of points for average/centroid calculation
        int clusterCount = 0; //start indexing from 0
        startPoint = detectedPoints->at(0); //initialize with the first point the lidar detects

        for (vector<Point2D>::iterator it = detectedPoints->begin(); it != detectedPoints->end(); it++) {
            Point2D p = *it; 
            startPoint.cluster = clusterCount; 
            if (startPoint.distance(p) <= radius * radius) { //if the points are close enough
                p.cluster = startPoint.cluster; //assigns the compared point the same clusterId
                IDPoints.push_back(p); //moves the compared point to another vector
            } else { //if it finds a point not close enough
                clusterCount += 1; //increase the count
                startPoint = p; //create a new starting point from the one just compared
                startPoint.cluster = clusterCount;
                IDPoints.push_back(startPoint); //add the newest point to the new vector
            }
            *it = p; //Why is this here?? I do not understand how this part works
        }
        // Initialise with zeroes
        for (int j = 0; j < clusterCount + 1; j++) { //Not exactly sure why +1 works. Without the +1, the program gives vector subscript failed
            nPoints.push_back(0);
            sumX.push_back(0.0);
            sumY.push_back(0.0);
            touchPoints.push_back(emptyPoint); //initializing with (0,0) points
        }
        // Iterate over points to append data to the vectors
        for (vector<Point2D>::iterator it = IDPoints.begin(); it != IDPoints.end(); it++) {
            auto& clusterPoints = *it;
            int clusterID = clusterPoints.cluster;
            nPoints[clusterID] += 1;
            sumX[clusterID] += clusterPoints.x;
            sumY[clusterID] += clusterPoints.y;     
        }
        // Compute the centroids/touch points
        for (vector<Point2D>::iterator c = begin(touchPoints); c != end(touchPoints) ; c++) {
            auto& touchPoint = *c;
            int clusterID = c - begin(touchPoints); //Why does this work??
            touchPoint.cluster = clusterID;
            touchPoint.x = sumX[clusterID] / nPoints[clusterID]; // sum over #points = average
            touchPoint.y = sumY[clusterID] / nPoints[clusterID]; 
        }
}
 

//Normalizes to between 0-1
static void screenToNormal(float x, float y, int cursorNumber, const ScreenSizeStruct screenSize, float& x_normal, float& y_normal, int& touchID) {
    x_normal = x / (-(screenSize.min_x) + screenSize.max_x);
    y_normal = y / (-(screenSize.min_y) + screenSize.max_y);
    touchID = cursorNumber;
}


//Simulate a left mouse click
void leftClick(int x, int y) {
    SetPhysicalCursorPos(x, y); //You can also do SetCursorPos(x, y); if you dont want to see the cursor move too.
    mouse_event(MOUSEEVENTF_LEFTDOWN, x, y, 0, 0); //left mouse button down
    mouse_event(MOUSEEVENTF_LEFTUP, x, y, 0, 0); //left mouse button up, simulates click
}


//Defines the screen size by showing the coordinates of the sides
ScreenSizeStruct screenSize(-0.42, 0.50, -0.70, -0.19); //screen distances from the reference of the lidar, have to do this manually right now
float x_touch, y_touch; //final x and y touch points
int cursorID; //final cursor ID
long totalTime; //Need this for calculating velocites and acceleration


/*************************************************************************************************************************/


int main(int argc, char *argv[]) {

    printf("Connor's Version \n \n");
    printf("__   ______  _     ___ ____    _    ____  \n");
    printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
    printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
    printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
    printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
    printf("\n");
    //fflush(stdout); //What does this do?
    std::string port;
    ydlidar::init(argc, argv);

    std::map<std::string, std::string> ports =
    ydlidar::YDlidarDriver::lidarPortList();
    std::map<std::string, std::string>::iterator it;

    if (ports.size() == 1) {
        port = ports.begin()->second;
    } else {
        int id = 0;
        for (it = ports.begin(); it != ports.end(); it++) {
        printf("%d. %s\n", id, it->first.c_str());
        id++;
        }
    }
    if (!ydlidar::ok()) {
        return 0;
    }
 
                                   /********* ALL SETTINGS SPECIFIC FOR YDLiDAR G4 ************/
    CYdLidar laser;
    //<! lidar port
          laser.setSerialPort("COM3");
    //<! lidar baudrate
          laser.setSerialBaudrate(230400);
    //<! fixed angle resolution
    laser.setFixedResolution(false);
    //<! rotate 180
    laser.setReversion(false); //rotate 180
    //<! Counterclockwise
    laser.setInverted(false);//ccw
    laser.setAutoReconnect(true);//hot plug
    //<! one-way communication
          laser.setSingleChannel(false);
    //<! tof lidar
          laser.setLidarType(TYPE_TRIANGLE);
    //unit: degrees
    laser.setMaxAngle(180); //Should probably base this on ScreenSizeStruct() but unsure how to correctly
    laser.setMinAngle(-180);
    //unit: m
          laser.setMinRange(0.01); //there seems to be a hardware (not software) limit of how close it can detect something
          laser.setMaxRange(64.0); //64 meters is the max range
    //unit: Hz
          laser.setScanFrequency(5.0); //This is how many times the lidar spins per second, the lidar frequency is actually 9000

    std::vector<float> ignore_array; //Don't know what this does
    ignore_array.clear();
    laser.setIgnoreArray(ignore_array);

  /*******************************************************************************************************************************/

    bool ret = laser.initialize();
    if (ret) {
        ret = laser.turnOn();
    }
    
    
    while (ret && ydlidar::ok()) {
    
    vector<Point2D> detectedPoints;
    vector<Point2D> touchPoints;
        
    bool hardwareError;
    float error = .005; //error for edges of the screen, make sure this is small
    float radius = .02; //size of finger or marker

    LaserScan scan;

        if (laser.doProcessSimple(scan, hardwareError)) {

            float scanTime; //time it takes to scan every time the sensor spins once
            float initTheta, initR, theta, r, initX, initY, x, y;
            bool inFOV, inScreen;

            for (auto laserPointIter = scan.points.begin(); laserPointIter != scan.points.end(); laserPointIter++) {
                auto& laserPoint = *laserPointIter;
                //scanTime = scan.config.scan_time;
                polarOutput(laserPoint, theta, r); //converts lidar vector/list into readable data
                limitScreenPolar(inFOV, theta, r, screenSize); //limits data to only show within fov
                if (inFOV) {
                    polarToCartesian(theta, r, initX, initY); //converts polar coordinates to cartesian 
                    cartesianToScreen(initX, initY, screenSize, error, x, y); //limits cartesian coordinates to be within a certain range and then shifts 0,0 to bottom left
                    if (x != NULL && y != NULL) { //push everything that is not 0,0 to the vector detectedPoints
                        Point2D Point(x, y); //creates a 2d coordinate from the x and y data
                        detectedPoints.push_back(Point); //adds the coordinate to the vector
                    } else;
                }
            }
            
            //totalTime += scanTime*1000; //This should equal a standard clock in ms
            
            if (detectedPoints.size() != 0) {
                cout << "\n\t\t Points Detected" << endl;
                touchClustering(&detectedPoints, touchPoints, radius); //finds the touch points through clusters
                for (vector<Point2D>::iterator it = touchPoints.begin(); it != touchPoints.end(); it++) {
                    auto& touchPoint = *it;
                    screenToNormal(touchPoint.x, touchPoint.y, touchPoint.cluster, screenSize, x_touch, y_touch, cursorID); 
                    cout << "Cursor # = " << cursorID << "\t\t x = " << x_touch << "\t y = " << y_touch << endl;
                    leftClick(x_touch*(1920/1.5), y_touch*(1080/1.5)); //1920*1080 screen, 150% scaling
                }
            } else {
                cout << "\n\t\t No Points Detected" << endl;
            }

        } else { //if statement fails
        fprintf(stderr, "Failed to get Lidar Data");
        laser.turnOff();
        laser.disconnecting();
        }

        cout << "\nReset Loop\n" << endl; //end of the while loop

    }
  
    laser.turnOff();
    laser.disconnecting();
    return 0;

}

