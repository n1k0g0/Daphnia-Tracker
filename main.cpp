#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <numeric>

using namespace std;
using namespace cv;


int main () {



    ///STEP 1. CAPTURING DATA GOT FROM DETECTOR

    string line;
    vector<vector<vector<int>>> rawCoords;

    ifstream myfile("/root/CLionProjects/daphnia/coords.txt");
    if (myfile.is_open())
    {
        int i = 0;
        while (getline(myfile, line))
        {
            vector<vector<int>> lineOfRawCoords = {{}};
            istringstream ss(line);
            string token;
            int j = 0;
            //cout << line << endl;

            cout << endl;
            vector<int> lineOfCoords1 = {};

            while (getline(ss, token, ',')) {
                //cout << token << " ";
                //cout << token << endl;
                int x, y;
                x = 0;
                y = 0;
                if (token.length() > 0 && token.at(0) == '"') {
                    token = token.substr(2, token.length());
                }
                if (token.length() > 0 && token.at(0) == ' ') {
                    token = token.substr(1, token.length() - 3);
                    //lineOfRawCoords.push_back({x, y});
                    //cout << lineOfRawCoords[j][0] << " " << lineOfRawCoords[j][1] << endl;
                }
                if (token.length() > 0 && token.at(token.length() - 1) == ')') {
                    token = token.substr(0, token.length() - 1);
                    //lineOfRawCoords.push_back({x, y});
                    //cout << lineOfRawCoords[j][0] << " " << lineOfRawCoords[j][1] << endl;
                }

                //cout << token << " ";
                lineOfCoords1.push_back(stoi(token));
                //cout << lineOfCoords1[j] << "; ";
                //cout << lineOfRawCoords.size() << endl;

                if (j % 2 == 1) {
                    lineOfRawCoords.push_back({lineOfCoords1[j-1], lineOfCoords1[j]});
                    cout << lineOfRawCoords[lineOfRawCoords.size() - 1][0] << " " << lineOfRawCoords[lineOfRawCoords.size() - 1][1] << "; ";
                }

                ++j;
            }

            rawCoords.push_back(lineOfRawCoords);
            cout << rawCoords[rawCoords.size() - 1].size() << endl;
            cout << endl << endl << endl;
            ++i;
        }
        myfile.close();

    } else{
        cout << "Unable to open file";
    }


    int i;
    for (i = 0; i < rawCoords.size()-1; ++i) {
        rawCoords[i].erase(rawCoords[i].begin());
        for (int j = 0; j < rawCoords[i].size()-1; ++j) {
            cout << rawCoords[i][j][0] << " " << rawCoords[i][j][1] << "; ";
        }
        cout << rawCoords[i].size();
        cout << endl;
    }


    vector<int> detectedPerFrame = {};
    for (i = 0; i < rawCoords.size()-1; ++i) {
        cout << rawCoords[i].size();
        detectedPerFrame.push_back(rawCoords[i].size());
        cout << endl;
    }


    double sum = accumulate(detectedPerFrame.begin(), detectedPerFrame.end(), 0.0);
    double mean = sum / detectedPerFrame.size();
    int intMean = static_cast<int>(mean);

    cout << endl << "The number of objects should be around: " << mean << endl;
    cout << endl << "Probably the number is: " << intMean << endl;



    /// STEP 1.5 AUXILIARY STRUCTURES

    struct Coord {
        int x;
        int y;
    };

    struct CoordsAndDistance {
        vector<int> coords;
        float dist;
    };










    /// STEP 2. TRACKING ALGORITHM


    vector<vector<Coord>> trackedCoords;
    vector<Coord> neighborsVector;


    //collecting objects from 1st frame
    Coord coord0{0,0};
    for (int j = 0; j < rawCoords[0].size(); ++j) {
        coord0.x = rawCoords[0][j][0];
        coord0.y = rawCoords[0][j][1];
        neighborsVector.push_back(coord0);
    }
    if (rawCoords[0].size() < intMean) {
        for (int j = rawCoords[0].size(); j < intMean; ++j) {
            coord0.x = j;
            coord0.y = j;
            neighborsVector.push_back(coord0);
        }
    }
    trackedCoords.push_back(neighborsVector);




    cout << trackedCoords[0].size() << endl;


    //collecting objects from other frames
    //for each frame
    for (int i = 1; i < rawCoords.size() - 1; ++i) {
        cout << i << endl;
        neighborsVector = {};


        //for each detected object of previous frame
        for (int j = 0; j < trackedCoords[i-1].size(); ++j) {


            vector<float> distances = {};
            for (auto coord : rawCoords[i]) {
                float a = (trackedCoords[i-1][j].x - coord[0])*(trackedCoords[i-1][j].x - coord[0]) + (trackedCoords[i-1][j].y - coord[1])*(trackedCoords[i-1][j].y - coord[1]);
                float b = (-trackedCoords[i-1][j].x - coord[0])*(-trackedCoords[i-1][j].x - coord[0]) + (-trackedCoords[i-1][j].y - coord[1])*(-trackedCoords[i-1][j].y - coord[1]);
                distances.push_back(min(a, b));
            }


                vector<CoordsAndDistance> coordsAndDists;
                for (int k = 0; k < rawCoords[i].size(); ++k) {
                    coordsAndDists.push_back({rawCoords[i][k], distances[k]});
                }

                sort(coordsAndDists.begin(), coordsAndDists.end(),
                               [](const auto& i, const auto& j) { return i.dist < j.dist; } );

                bool appended = false;
                int idx = 0;

                while (appended == false) {
                    float distToClosest = 100.0;
                    if (neighborsVector.size() > 0) {
                        vector<float> distToAppendedPoints{};
                        for (auto coord0 : neighborsVector) {
                            distToAppendedPoints.push_back((coordsAndDists[idx].coords[0] - coord0.x)*(coordsAndDists[idx].coords[0] - coord0.x) + (coordsAndDists[idx].coords[1] - coord0.y)*(coordsAndDists[idx].coords[1] - coord0.y));
                        }
                        vector<float>::iterator result = min_element(distToAppendedPoints.begin(), distToAppendedPoints.end());

                        distToClosest = distToAppendedPoints[distance(distToAppendedPoints.begin(), result)];
                    }
                        if (distToClosest > 5) {
                            if (coordsAndDists[idx].dist < 50) {
                                neighborsVector.push_back({abs(coordsAndDists[idx].coords[0]), abs(coordsAndDists[idx].coords[1])});
                            } else {
                                neighborsVector.push_back({-abs(trackedCoords[i-1][j].x), -abs(trackedCoords[i-1][j].y)});
                            }
                            appended = true;
                        }
                        if (coordsAndDists.size() > idx + 1) {
                            idx += 1;
                        } else {
                            appended = true;
                        }
                }
        }



        if (rawCoords[i].size() < intMean) {
            for (int j = 0; j < rawCoords[i].size(); ++j) {

                bool isInNeighbors = false;
                for (auto neighbor : neighborsVector) {
                    float d = (neighbor.x - rawCoords[i][j][0])*(neighbor.x - rawCoords[i][j][0]) + (neighbor.y - rawCoords[i][j][1])*(neighbor.y - rawCoords[i][j][1]);
                    if (d < 0.1) {
                        isInNeighbors = true;
                    }
                }

                if (isInNeighbors == false) {
                    vector<float> dists;
                    for (int k = 0; k < neighborsVector.size(); ++k) {
                        dists.push_back((-rawCoords[i][j][0] - neighborsVector[k].x)*(-rawCoords[i][j][0] - neighborsVector[k].x) + (-rawCoords[i][j][1] - neighborsVector[k].y)*(-rawCoords[i][j][1] - neighborsVector[k].y));
                    }
                    vector<float>::iterator result = min_element(dists.begin(), dists.end());
                    float minDist = dists[distance(dists.begin(), result)];
                    if ((minDist < 10) && (neighborsVector[distance(dists.begin(), result)].x < 0)) {
                        neighborsVector[distance(dists.begin(), result)] = Coord{rawCoords[i][j][0], rawCoords[i][j][1]};
                    }
                }
            }
        }


        if (rawCoords[i].size() < intMean) {
            for (int j = 0; j < neighborsVector.size(); ++j) {
                if (neighborsVector[j].x < 0) {
                    vector<float> dists;
                    for (int k = 0; k < rawCoords[i].size(); ++k) {
                        dists.push_back((-rawCoords[i][k][0] - neighborsVector[j].x)*(-rawCoords[i][k][0] - neighborsVector[j].x) + (-rawCoords[i][k][1] - neighborsVector[j].y)*(-rawCoords[i][k][1] - neighborsVector[j].y));
                    }
                    vector<float>::iterator result = min_element(dists.begin(), dists.end());
                    float minDist = dists[distance(dists.begin(), result)];



                    bool isInNeighbors = false;
                    for (auto neighbor : neighborsVector) {
                        float d = (neighbor.x - rawCoords[i][distance(dists.begin(), result)][0])*(neighbor.x - rawCoords[i][distance(dists.begin(), result)][0]) + (neighbor.y - rawCoords[i][distance(dists.begin(), result)][1])*(neighbor.y - rawCoords[i][distance(dists.begin(), result)][1]);
                        if (d < 0.1) {
                            isInNeighbors = true;
                        }
                    }



                    if ((minDist > 50) && (isInNeighbors == false)) {
                        neighborsVector[j] = Coord{rawCoords[i][distance(dists.begin(), result)][0], rawCoords[i][distance(dists.begin(), result)][1]};
                    }
                }
            }
        }


        if ((neighborsVector.size() < intMean) && (rawCoords[i].size() >= neighborsVector.size())) {
            for (int j = neighborsVector.size(); j < min(float(intMean), float(rawCoords[i].size())); ++j) {
                vector<float> dists;
                for (int k = 0; k < neighborsVector.size(); ++k) {
                    dists.push_back((-rawCoords[i][j][0] - neighborsVector[k].x)*(-rawCoords[i][j][0] - neighborsVector[k].x) + (-rawCoords[i][j][1] - neighborsVector[k].y)*(-rawCoords[i][j][1] - neighborsVector[k].y));
                }
                vector<float>::iterator result = min_element(dists.begin(), dists.end());
                float minDist = dists[distance(dists.begin(), result)];


                bool isInNeighbors = false;
                for (auto neighbor : neighborsVector) {
                    float d = (neighbor.x - rawCoords[i][distance(dists.begin(), result)][0])*(neighbor.x - rawCoords[i][distance(dists.begin(), result)][0]) + (neighbor.y - rawCoords[i][distance(dists.begin(), result)][1])*(neighbor.y - rawCoords[i][distance(dists.begin(), result)][1]);
                    if (d < 0.1) {
                        isInNeighbors = true;
                    }
                }


                if (isInNeighbors == false) {
                    neighborsVector.push_back(Coord{rawCoords[i][j][0], rawCoords[i][j][1]});
                }
            }
        }


        for (auto neighbour : neighborsVector) {
            cout << "(" << neighbour.x << ", " << neighbour.y << "); ";
        }
        trackedCoords.push_back(neighborsVector);
        cout << endl;

    }





    /// STEP 4. DISPLAYING VIDEO

    VideoCapture cap("/root/CLionProjects/daphnia/Cohort44_Large_Light.avi");
    int frameW = cap.get(CAP_PROP_FRAME_WIDTH);
    int frameH = cap.get(CAP_PROP_FRAME_HEIGHT);
    VideoWriter video("/root/CLionProjects/daphnia/out.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, Size(frameW, frameH));
    // Check if camera opened successfully
    if(!cap.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return -1;
    }

    int frameCount = 0;
    while(1){

        Mat frame;

        // Capture frame-by-frame
        cap >> frame;


        Mat roi = frame(Range(55, 1024), Range(70, 1150));

        // If the frame is empty, break immediately
        if (frame.empty())
            break;

        ///SHOWING RAW DETECTED VALUES W/O TRACKING
        for (int j = 0; j < rawCoords[frameCount].size(); ++j) {
            circle(roi, Point(rawCoords[frameCount][j][0], rawCoords[frameCount][j][1]), 5, Scalar(255*j / rawCoords[frameCount].size(), 15*j, 255 - 255*j / rawCoords[frameCount].size()), FILLED, LINE_8);
            //putText(roi, to_string(j+1), Point(rawCoords[frameCount][j][0] - 10, rawCoords[frameCount][j][1] - 10), FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2);
        }

        ///SHOWING OBJECTS AFTER TRACKING
        for (int j = 0; j < trackedCoords[frameCount].size(); ++j) {
            circle(roi, Point(trackedCoords[frameCount][j].x, trackedCoords[frameCount][j].y), 2, Scalar(100 + 10*j, 15*j, 255 - 10*j), FILLED, LINE_8);
            putText(roi, to_string(j+1), Point(trackedCoords[frameCount][j].x - 10, trackedCoords[frameCount][j].y - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 2);
        }


        // Display the resulting frame
        imshow( "Frame", frame );
        video.write(frame);

        // Press  ESC on keyboard to exit
        char c=(char)waitKey(25);
        if(c==27)
            break;

        ++frameCount;
    }

    // When everything done, release the video capture object
    cap.release();
    video.release();
    // Closes all the frames
    destroyAllWindows();



    return 0;
}