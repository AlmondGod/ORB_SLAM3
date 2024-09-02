/*
xvfb-run ./mono_custom2 /home/almondgod/ORB_SLAM3/Vocabulary/ORBvoc.txt raspicam_v2_640x480.yaml
*/
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<thread>
#include<mutex>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void ProcessNewImages(ORB_SLAM3::System& SLAM, const string& imagePath, const string& timestampPath);

cv::Mat DrawSLAMInfo(const cv::Mat &im, const cv::Mat &Tcw);

bool gIsRunning = true;
mutex gMutex;

int main(int argc, char **argv)
{  
    if(argc != 3)
    {
        cout << endl << "Usage: ./mono_custom path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    cout << endl << "slam system initialized" << endl;

    thread processing_thread(ProcessNewImages, ref(SLAM), 
                             "saved-images", 
                             "saved-images/timestamps.txt");

    cout << endl << "processing thread started" << endl;                         

    // Main loop to keep the program running
    // while(true)
    // {
    //     this_thread::sleep_for(chrono::seconds(1));
        
    //     // Check for exit condition (you might want to implement a proper way to exit)
    //     char input;
    //     if (cin.get(input) && input == 'q')
    //     {
    //         lock_guard<mutex> lock(gMutex);
    //         gIsRunning = false;
    //         break;
    //     }
    // }

    processing_thread.join();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    SLAM.SaveMapPoints("MapPoints.txt");

    return 0;
}

void ProcessNewImages(ORB_SLAM3::System& SLAM, const string& imagePath, const string& timestampPath)
{
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    int lastProcessedIndex = -1;

    // while(true)
    // {
        // {
        //     lock_guard<mutex> lock(gMutex);
        //     if (!gIsRunning) break;
        // }

        cout << endl << "loading images" << endl; 
        // Load new images
        LoadImages(imagePath, timestampPath, vstrImageFilenames, vTimestamps);

        cout << endl << "images loaded" << endl; 

        // Process new images
        for(int ni = lastProcessedIndex + 1; ni < vstrImageFilenames.size(); ni++)
        {
            cv::Mat im = cv::imread(vstrImageFilenames[ni], cv::IMREAD_UNCHANGED);
            double tframe = vTimestamps[ni];

            if(im.empty())
            {
                cout << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
                continue;
            }

            if(!im.empty())
            {
                cout << "Successfully loaded image: " << vstrImageFilenames[ni] 
                    << ", Size: " << im.cols << "x" << im.rows 
                    << ", Channels: " << im.channels() << endl;
            }

            // Check image format
            string format;
            if(im.channels() == 1)
                format = "Grayscale";
            else if(im.channels() == 3)
                format = (im.type() == CV_8UC3) ? "RGB" : "BGR";
            else
                format = "Unknown";

            cout << "Image: " << vstrImageFilenames[ni] 
                << ", Size: " << im.cols << "x" << im.rows 
                << ", Channels: " << im.channels() 
                << ", Format: " << format << endl;

            // Pass the image to the SLAM system
            // SLAM.TrackMonocular(im, tframe);
            // cv::Mat im_gray;
            // cv::cvtColor(im, im_gray, cv::COLOR_RGB2GRAY);
            try {
                SLAM.TrackMonocular(im, tframe);
            } catch (const std::exception& e) {
                std::cout << "Exception in TrackMonocular: " << e.what() << std::endl;
            } catch (...) {
                std::cout << "Unknown exception in TrackMonocular" << std::endl;
            }

            lastProcessedIndex = ni;
        }

        // Wait before checking for new images
        // this_thread::sleep_for(chrono::milliseconds(100));
    // }
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    cout << "Loading images from: " << strImagePath << endl;

    ifstream fTimes(strPathTimes.c_str());
    if (!fTimes.is_open()) {
        cout << "Error: Could not open file " << strPathTimes << endl;
        return;
    }

    vTimeStamps.clear();
    vstrImages.clear();

    cout << "Loading timestamps from: " << strPathTimes << endl;

    string s;
    while (getline(fTimes, s))
    {
        if (!s.empty())
        {
            stringstream ss(s);
            double t;
            string sRGB;

            if (ss >> t >> sRGB)
            {
                vTimeStamps.push_back(t);
                vstrImages.push_back(strImagePath + "/" + sRGB);
            }
            else
            {
                cout << "Error parsing line: " << s << endl;
            }
        }
    }
    
    cout << "Loaded " << vstrImages.size() << " images" << endl;
}