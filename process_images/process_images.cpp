#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ORBextractor.h"

using namespace std;

// Function to get all image files in a directory
vector<string> getImageFiles(const string& folderPath) {
    vector<string> fileNames;
    cv::String path(folderPath);
    vector<cv::String> fn;
    cv::glob(path + "/*.png", fn, false);

    for (const auto& f : fn) {
        fileNames.push_back(f);
    }
    return fileNames;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        cerr << "Usage: ./process_images <path_to_images>" << endl;
        return 1;
    }

    string folderPath = argv[1];
    vector<string> imageFiles = getImageFiles(folderPath);

    if (imageFiles.empty()) {
        cerr << "No images found in " << folderPath << endl;
        return 1;
    }

    cout << "Found " << imageFiles.size() << " images." << endl;

    // ORB Configuration (Standard ORB-SLAM3 defaults)
    int nFeatures = 4000;
    float fScaleFactor = 1.2f;
    int nLevels = 13;
    int fIniThFAST = 12;
    int fMinThFAST = 7;

    // Initialize ORB Extractor
    ORB_SLAM3::ORBextractor* mpORBextractor = new ORB_SLAM3::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    for (const string& imagePath : imageFiles) {
        cout << "Processing: " << imagePath << endl;
        cv::Mat im = cv::imread(imagePath, cv::IMREAD_UNCHANGED);
        if (im.empty()) {
            cerr << "Failed to load " << imagePath << endl;
            continue;
        }

        cv::Mat imGray;
        if (im.channels() == 3) {
            cv::cvtColor(im, imGray, cv::COLOR_BGR2GRAY);
        } else if (im.channels() == 4) {
            cv::cvtColor(im, imGray, cv::COLOR_BGRA2GRAY);
        } else {
            imGray = im;
        }

        vector<cv::KeyPoint> mvKeys;
        cv::Mat mDescriptors;

        // Compute ORB features
        vector<int> vLapping = {0, 10000}; // Dummy lapping area
        (*mpORBextractor)(imGray, cv::Mat(), mvKeys, mDescriptors, vLapping);

        // Create Mask
        cv::Mat mask = cv::Mat::zeros(im.size(), CV_8UC1);
        for (const auto& kp : mvKeys) {
            int x = cvRound(kp.pt.x);
            int y = cvRound(kp.pt.y);
            if (x >= 0 && x < mask.cols && y >= 0 && y < mask.rows)
                mask.at<uchar>(y, x) = 255;
        }

        // Generate output filename
        string filename = imagePath;
        size_t last_slash = filename.find_last_of("/\\");
        if (last_slash != string::npos) filename = filename.substr(last_slash + 1);

        string outputFilename = "feat_" + filename + ".png";
        
        cv::imwrite(outputFilename, mask);
    }

    delete mpORBextractor;
    cout << "Done." << endl;

    return 0;
}
