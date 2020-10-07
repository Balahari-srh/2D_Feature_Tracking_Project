# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load.
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed.
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson.
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures.

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning.

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

_____________________________________________________________________________________________________________________________________________

# Performance Evaluation
Detector - Line number 89 in MidTermProject_Camera_Student.cpp
Descriptor - Line number 160 in MidTermProject_Camera_Student.cpp

## Task 1
- Loading procedure of images.
```C++
  DataFrame frame;
  frame.cameraImg = imgGray;

  if(dataBuffer.size()> dataBufferSize)
  {
      dataBuffer.erase(dataBuffer.begin());
  }
  dataBuffer.push_back(frame);
```
## Task 2
- HARRIS, FAST, BRISK, ORB, AKAZE and SIFT detectors are implemented in
- For evaluation of different Detectors, it has to be altered in line number 160 in MidTermProject_Camera_Student.cpp
```C++
string detectorType = detector[0]; //{"SHITOMASI","HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
```
## Task 3
- Bounding box on the previous vehicle to crop the image to evaluvate only on ROI.
```C++
   bool bFocusOnVehicle = true;
   cv::Rect vehicleRect(535, 180, 180, 150);
   if (bFocusOnVehicle)
   {
       // ...
       vector<cv::KeyPoint> keypoints_in_rect;
       for (auto point : keypoints) {
           if (vehicleRect.contains(point.pt))
           {
               point.size = static_cast<float>(round(point.size*10)/10);
               keypoints_in_rect.push_back(point);
           }
       }

       keypoints.swap(keypoints_in_rect);

       cout << "Number of keypoint in ROI =" << keypoints.size() << endl;
   }
```
## Task 4
- BRISK, BRIEF, ORB, FREAK, AKAZE and SIFT are implemented in matching2D_Student.cpp
- The default parameters are used as per openCV documentation for each of the keypoint descriptors.
- For evaluation of different descriptor, it has to be altered in line number 160 in MidTermProject_Camera_Student.cpp
```C++
string descriptorType = descriptor[0]; // {"BRISK","BRIEF","ORB","FREAK","AKAZE","SIFT"}
```
## Task 5
- Added FLANN based matching along with Brute Force Matching(KNN discussed in task 6)
(Implemented in matching2D_Student.cpp)
```C++
    if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F||descRef.type()!=CV_32F)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }
        //... TODO : implement FLANN matching
        cout << "FLANN matching";
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }
```

## Task 6
- Implementing the descriptor distance ratio test as a filtering method to remove bad keypoint matches.
(Implemented in matching2D_Student.cpp)
```C++
    if (selectorType.compare("SEL_KNN") == 0)
    {   // k nearest neighbors (k=2)
        std::vector<std::vector<cv::DMatch>> knn_matches;
        // TODO : implement k-nearest-neighbor matching
        double t = (double)cv::getTickCount();
        matcher->knnMatch(descSource, descRef, knn_matches,2); // Finds the best match for each descriptor in desc1
        // TODO : filter matches using descriptor distance ratio test
        const float ratio_thresh = 0.8f;
        for(size_t it =0;it<knn_matches.size();it++)
        {
            if(knn_matches[it][0].distance<ratio_thresh*knn_matches[it][1].distance)
            {
                matches.push_back(knn_matches[it][0]);
            }
        }
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (KNN) with n=" << knn_matches.size()-matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
    }
```

## Task 7

- Number of Keypoints on the preceeding vehicle for all the 10 images.

Detector | Keypoints in ROI
---------|-----------------
ShiTomasi|1179
HARRIS   |1084
FAST     |4094
BRISK    |2762
Orb      |1161
AKAZE    |1670
SIFT     |1386

## Task 8
- Number of matched Keypoints for all 10 images using all possible combinations of detectors and descriptors. In matching step, BF approach with descriptor distance ratio is set to 0.8

## Task 9
- Time taken for keypoint detection and descriptor extraction is available on the spread sheet file in the Performance Evaluation folder(sheet name- Task 9).
- Top 3 detector/descriptor combination for detecting keypoints on vehicles.

Detector  |   Descriptor
----------|--------------
FAST      |   Brief
FAST      |   Orb
FAST      |   Brisk

FAST + Brief achieves outperforms other combination of Detector-Descriptor combinations with minimal processing time(result of TASK 9) as well as with the most number of matches(result of TASK 8).

The overall data used to evaluate task 7,8 and 9 is available at [Performance Evaluation!](https://github.com/Balahari-srh/2D_Feature_Tracking_Project/blob/master/Performance%20Evaluation/Final%20Results.ods)
