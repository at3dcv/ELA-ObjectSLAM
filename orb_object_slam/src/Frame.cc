/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

// VARIABLE DEFINITIONS
// mvKeyUn = undistorted keypoints

#include "Frame.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "ObjDetectionHelper.h"

#include "Converter.h"
#include "ORBmatcher.h"
#include "ORBextractor.h"
#include <thread>

#include "Parameters.h"

#include "detect_3d_cuboid/matrix_utils.h"

#include <fstream>
#include <iostream>
#include <string>

#include "tictoc_profiler/profiler.hpp"

// Used in Frame::DetectMovingKeypoints
cv::Mat imGrayPre;
std::vector<cv::Point2f> prepoint, nextpoint;
std::vector<cv::Point2f> F_prepoint, F_nextpoint;
std::vector<cv::Point2f> F2_prepoint, F2_nextpoint;
// Used in OpenCV::calcOpticalFlowPyrLK
std::vector<uchar> state;
std::vector<float> err;
std::vector<std::vector<cv::KeyPoint>> mvKeysPre;

namespace ORB_SLAM2
{

long unsigned int Frame::nNextId = 0;
bool Frame::mbInitialComputations = true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame()
{
}

//Copy Constructor
Frame::Frame(const Frame &frame)
    : mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
      mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
      mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
      mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn), mvuRight(frame.mvuRight),
      mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
      mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
      mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
      mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
      mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
      mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
      mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
{
    for (int i = 0; i < FRAME_GRID_COLS; i++)
        for (int j = 0; j < FRAME_GRID_ROWS; j++)
            mGrid[i][j] = frame.mGrid[i][j];

    if (!frame.mTcw.empty())
        SetPose(frame.mTcw);

    if (whether_detect_object)
        raw_img = frame.raw_img.clone();
    if (whether_dynamic_object)
    {
        KeysStatic = frame.KeysStatic;
        keypoint_associate_objectID = frame.keypoint_associate_objectID;
        numobject = frame.numobject;
        objmask_img = frame.objmask_img;
        mvKeysHarris = frame.mvKeysHarris;
        mvpMapPointsHarris = frame.mvpMapPointsHarris;
        keypoint_associate_objectID_harris = frame.keypoint_associate_objectID_harris;
    }
}

// for stereo
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor *extractorLeft, ORBextractor *extractorRight, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    : mpORBvocabulary(voc), mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
      mpReferenceKF(static_cast<KeyFrame *>(NULL))
{
    // Frame ID
    mnId = nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    thread threadLeft(&Frame::ExtractORB, this, 0, imLeft);
    thread threadRight(&Frame::ExtractORB, this, 1, imRight);
    threadLeft.join();
    threadRight.join();

    if (mvKeys.empty())
        return;

    N = mvKeys.size();

    UndistortKeyPoints();

    ComputeStereoMatches();

    mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));
    mvbOutlier = vector<bool>(N, false);

    // This is done only for the first Frame (or after a change in the calibration)
    if (mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
        mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

        fx = K.at<float>(0, 0);
        fy = K.at<float>(1, 1);
        cx = K.at<float>(0, 2);
        cy = K.at<float>(1, 2);
        invfx = 1.0f / fx;
        invfy = 1.0f / fy;

        mbInitialComputations = false;
    }

    mb = mbf / fx;

    AssignFeaturesToGrid();
}

// for RGBD
Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    : mpORBvocabulary(voc), mpORBextractorLeft(extractor), mpORBextractorRight(static_cast<ORBextractor *>(NULL)),
      mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId = nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0, imGray);

    N = mvKeys.size();

    if (mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoFromRGBD(imDepth);

    mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));
    mvbOutlier = vector<bool>(N, false);

    // This is done only for the first Frame (or after a change in the calibration)
    if (mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
        mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

        fx = K.at<float>(0, 0);
        fy = K.at<float>(1, 1);
        cx = K.at<float>(0, 2);
        cy = K.at<float>(1, 2);
        invfx = 1.0f / fx;
        invfy = 1.0f / fy;

        mbInitialComputations = false;
    }

    mb = mbf / fx;

    AssignFeaturesToGrid();
}

// for monocular
Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf,
             const float &thDepth)
    : mpORBvocabulary(voc), mpORBextractorLeft(extractor), mpORBextractorRight(static_cast<ORBextractor *>(NULL)),
      mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth), mpReferenceKF(static_cast<KeyFrame *>(NULL))
{
    // Frame ID
    mnId = nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();
    
    ExtractORB(0, imGray); // orb detector   change mvKeys, mDescriptors

    // AC: Copied from DS-SLAM
    if(imGrayPre.data)
    {
        DetectMovingKeypoints(imGray);
        imGrayPre = imGray.clone();
    }
    else
    {
        imGrayPre = imGray.clone();
    }
    
    // AC: here was the dynamic object removal process, however, it only takes a segmented map
    // AC: with only dynamic objects
    // if (whether_dynamic_object)
    // {
    //     // AC: commented out as different naming convention
    //     // char frame_index_c[256];
    //     // sprintf(frame_index_c, "%04d", (int)mnId); // format into 4 digit
    //     std::string pred_mask_img_name = base_data_folder + "rcnn_labelmap_3dmatched/" + to_string(mnId) + "_maskmap.png";
    //     objmask_img = cv::imread(pred_mask_img_name, CV_LOAD_IMAGE_UNCHANGED); // uint8  sometimes read image might take long time....

    //     KeysStatic = vector<bool>(mvKeys.size(), true); // all points are static now.
    //     keypoint_associate_objectID = vector<int>(mvKeys.size(), -1);
    //     numobject = 0;

    //     for (size_t i = 0; i < mvKeys.size(); i++)
    //     {
    //         int maskval = int(objmask_img.at<uint8_t>(mvKeys[i].pt.y, mvKeys[i].pt.x));
    //         KeysStatic[i] = (maskval == 0); //0 is background, static >0 object id
    //         numobject = max(numobject, maskval);
    //         if (maskval > 0)
    //             keypoint_associate_objectID[i] = maskval - 1;
    //     }
    // }

    N = mvKeys.size();

    if (mvKeys.empty())
        return;
        
    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N, -1);
    mvDepth = vector<float>(N, -1);

    mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));
    mvbOutlier = vector<bool>(N, false);

    // This is done only for the first Frame (or after a change in the calibration)
    if (mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
        mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

        fx = K.at<float>(0, 0);
        fy = K.at<float>(1, 1);
        cx = K.at<float>(0, 2);
        cy = K.at<float>(1, 2);
        invfx = 1.0f / fx;
        invfy = 1.0f / fy;

        mbInitialComputations = false;
    }

    mb = mbf / fx;

    AssignFeaturesToGrid();
}

void Frame::FilterOutMovingPoints(cv::Mat &imRGB, const cv::Mat &imGray, int frame_id)
{
    mCurrentObjDetection = ObjDetectionHelper();
    char frame_index_c[256];
    sprintf(frame_index_c, "%04d", (int)frame_id); // format into 4 digit
    mCurrentObjDetection.ReadFile(base_data_folder + "results/" + frame_index_c + "_mrcnn.txt");
    mCurrentBBoxes = mCurrentObjDetection.GetBBoxesWithPerson();

    if (!T_M.empty() && mCurrentBBoxes.size() > 0) {
        CheckMovingKeyPoints(imGray, mCurrentBBoxes);
    }
}

// AC: The generated keypoints are ONLY used to determine whether a frame has a moving object or not!
// TODO: instead of creating new keypoints, use old ones
void Frame::DetectMovingKeypoints(const cv::Mat &imgray)
{
    // Clear the previous data
	F_prepoint.clear();
	F_nextpoint.clear();
	F2_prepoint.clear();
	F2_nextpoint.clear();
	T_M.clear();

	// Detect dynamic target and ultimately optput the T matrix
    cv::goodFeaturesToTrack(imGrayPre, prepoint, 1000, 0.01, 8, cv::Mat(), 3, true, 0.04);
    cv::cornerSubPix(imGrayPre, prepoint, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
	cv::calcOpticalFlowPyrLK(imGrayPre, imgray, prepoint, nextpoint, state, err, cv::Size(22, 22), 5, cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.01));
    // AC: output of state: output status vector (of unsigned chars); each element of the vector is set to 1 if the flow for the corresponding features has been found, otherwise, it is set to 0.
	// AC: check whether the KP is too far to the edge of the image or the difference to each other is too large

    for (int i = 0; i < state.size(); i++)
    {
        if(state[i] != 0)
        {
            int dx[10] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };
            int dy[10] = { -1, -1, -1, 0, 0, 0, 1, 1, 1 };
            int x1 = prepoint[i].x, y1 = prepoint[i].y;
            int x2 = nextpoint[i].x, y2 = nextpoint[i].y;
            if ((x1 < limit_edge_corner || x1 >= imgray.cols - limit_edge_corner || x2 < limit_edge_corner || x2 >= imgray.cols - limit_edge_corner
            || y1 < limit_edge_corner || y1 >= imgray.rows - limit_edge_corner || y2 < limit_edge_corner || y2 >= imgray.rows - limit_edge_corner))
            {
                state[i] = 0;
                continue;
            }
            double sum_check = 0;
            for (int j = 0; j < 9; j++)
                sum_check += abs(imGrayPre.at<uchar>(y1 + dy[j], x1 + dx[j]) - imgray.at<uchar>(y2 + dy[j], x2 + dx[j]));
            if (sum_check > limit_of_check) state[i] = 0;
            // If KP is not discarded (state == 0), push KP to F_[KP]
            if (state[i])
            {
                F_prepoint.push_back(prepoint[i]);
                F_nextpoint.push_back(nextpoint[i]);
            }
        }
    }

    // F-Matrix
    cv::Mat mask = cv::Mat(cv::Size(1, 300), CV_8UC1);
    cv::Mat F = cv::findFundamentalMat(F_prepoint, F_nextpoint, mask, cv::FM_RANSAC, 0.1, 0.99);
    for (int i = 0; i < mask.rows; i++)
    {
        if (mask.at<uchar>(i, 0) == 0);
        else
        {
            // Circle(pre_frame, F_prepoint[i], 6, Scalar(255, 255, 0), 3);
            double A = F.at<double>(0, 0)*F_prepoint[i].x + F.at<double>(0, 1)*F_prepoint[i].y + F.at<double>(0, 2);
            double B = F.at<double>(1, 0)*F_prepoint[i].x + F.at<double>(1, 1)*F_prepoint[i].y + F.at<double>(1, 2);
            double C = F.at<double>(2, 0)*F_prepoint[i].x + F.at<double>(2, 1)*F_prepoint[i].y + F.at<double>(2, 2);
            double dd = fabs(A*F_nextpoint[i].x + B*F_nextpoint[i].y + C) / sqrt(A*A + B*B); //Epipolar constraints
            // Check whether the distance of a matched point to its epipolar line is within a certain threshold
            if (dd <= 0.1)
            {
                F2_prepoint.push_back(F_prepoint[i]);
                F2_nextpoint.push_back(F_nextpoint[i]);
            }
        }
    }

    F_prepoint = F2_prepoint;
    F_nextpoint = F2_nextpoint;

    for (int i = 0; i < prepoint.size(); i++)
    {
        if (state[i] != 0)
        {
            // AC: formula of DS SLAM paper (3)
            double A = F.at<double>(0, 0)*prepoint[i].x + F.at<double>(0, 1)*prepoint[i].y + F.at<double>(0, 2);
            double B = F.at<double>(1, 0)*prepoint[i].x + F.at<double>(1, 1)*prepoint[i].y + F.at<double>(1, 2);
            double C = F.at<double>(2, 0)*prepoint[i].x + F.at<double>(2, 1)*prepoint[i].y + F.at<double>(2, 2);
            double dd = fabs(A*nextpoint[i].x + B*nextpoint[i].y + C) / sqrt(A*A + B*B);

            if (dd <= limit_dis_epi) continue;
            T_M.push_back(nextpoint[i]);
        }
    }
}

// TODO: Add semantic mask
void Frame::CheckMovingKeyPoints(const cv::Mat &imGray, const std::vector<std::vector<float > > mCurrentBBoxes)
{
    std::vector<bool> objectsAreMoving = vector<bool>(mCurrentBBoxes.size(), false);

    // Make further judgment
    // AC: Check whether an object is moving
    for (int j = 0; j < mCurrentBBoxes.size(); j++)
    {
        float bbLeft = (float)mCurrentBBoxes[j][1];
        float bbTop = (float)mCurrentBBoxes[j][2];
        float bbRight = bbLeft + mCurrentBBoxes[j][3];
        float bbBottom = bbTop - mCurrentBBoxes[j][4];

        int movingKeypointCounter = 0;

        for (int i = 0; i < T_M.size(); i++)
        {
            // AC: Check whether keypoint is in bounding box
            if((int)T_M[i].x >= bbLeft && (int)T_M[i].x <= bbRight && (int)T_M[i].y <= bbTop && (int)T_M[i].y >= bbBottom)
            {
                movingKeypointCounter++;
                // AC: if two keypoints are in object, it is considered moving
                if (movingKeypointCounter > 1)
                {
                    objectsAreMoving[j] = 1;
                    break;
                }
            }
        }
    }

    // AC: If yes, erase points in given objects
    KeysStatic = vector<bool>(mvKeys.size(), true); // all points are static now.
    keypoint_associate_objectID = vector<int>(mvKeys.size(), -1);
    numobject = 0;

    for (int j = 0; j < mCurrentBBoxes.size(); j++)
    {
        if (objectsAreMoving[j])
        {
            float bbLeft = (float)mCurrentBBoxes[j][1];
            float bbTop = (float)mCurrentBBoxes[j][2];
            float bbRight = bbLeft + mCurrentBBoxes[j][3];
            float bbBottom = bbTop - mCurrentBBoxes[j][4];

            for (size_t i = 0; i < mvKeys.size(); i++)
            {
                if ((int)mvKeys[i].pt.x >= bbLeft && (int)mvKeys[i].pt.x <= bbRight && (int)mvKeys[i].pt.y <= bbTop && (int)mvKeys[i].pt.y >= bbBottom)
                {
                    KeysStatic[i] = 0; //0 is background, static >0 object id
                    numobject = max(numobject, 1);
                    keypoint_associate_objectID[i] = 0;
                }
            }
        }
    }

    if (remove_dynamic_features)
    {
        std::vector<cv::KeyPoint> mvKeys_cp;
        cv::Mat mDescriptors_cp;

        for (int l = 0; l < KeysStatic.size(); l++)
        {
            if (KeysStatic[l]) {
                mvKeys_cp.push_back(mvKeys[l]);
                mDescriptors_cp.push_back(mDescriptors.row(l));
            }
        }
        cout << "Kept " << mvKeys_cp.size() << "/" << mvKeys.size() << " keypoints" << endl;
        mvKeys = mvKeys_cp;
        mDescriptors = mDescriptors_cp.clone();
        KeysStatic = vector<bool>(mvKeys.size(), true);
        return;
    }

    // AC: For debugging
    int dynamicKeypointsCounter = 0;
    for (int k = 0; k < KeysStatic.size(); k++)
    {
        if (!KeysStatic[k]) {
            dynamicKeypointsCounter++;
        }
    }

    cout << "Found " << dynamicKeypointsCounter << "/" << mvKeys.size() << " dynamic keypoints" << endl;
}

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f * N / (FRAME_GRID_COLS * FRAME_GRID_ROWS);
    for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
        for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
            mGrid[i][j].reserve(nReserve);

    for (int i = 0; i < N; i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if (PosInGrid(kp, nGridPosX, nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if (flag == 0)
        (*mpORBextractorLeft)(im, cv::Mat(), mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im, cv::Mat(), mvKeysRight, mDescriptorsRight);
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{
    mRcw = mTcw.rowRange(0, 3).colRange(0, 3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0, 3).col(3);
    mOw = -mRcw.t() * mtcw;

    mTwc = cv::Mat::eye(4, 4, CV_32F);
    mRwc.copyTo(mTwc.rowRange(0, 3).colRange(0, 3));
    mOw.copyTo(mTwc.rowRange(0, 3).col(3));
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw * P + mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY = Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if (PcZ < 0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f / PcZ;
    const float u = fx * PcX * invz + cx;
    const float v = fy * PcY * invz + cy;

    if (u < mnMinX || u > mnMaxX)
        return false;
    if (v < mnMinY || v > mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P - mOw;
    const float dist = cv::norm(PO);

    if (dist < minDistance || dist > maxDistance)
        return false;

    // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn) / dist;

    if (viewCos < viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist, mfLogScaleFactor);

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf * invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel = nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0, (int)floor((x - mnMinX - r) * mfGridElementWidthInv));
    if (nMinCellX >= FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS - 1, (int)ceil((x - mnMinX + r) * mfGridElementWidthInv));
    if (nMaxCellX < 0)
        return vIndices;

    const int nMinCellY = max(0, (int)floor((y - mnMinY - r) * mfGridElementHeightInv));
    if (nMinCellY >= FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1, (int)ceil((y - mnMinY + r) * mfGridElementHeightInv));
    if (nMaxCellY < 0)
        return vIndices;

    const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

    for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
    {
        for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if (vCell.empty())
                continue;

            for (size_t j = 0, jend = vCell.size(); j < jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if (KeysStatic.size() > 0 && !KeysStatic[vCell[j]]) // this function is for static point
                    continue;
                if (bCheckLevels)
                {
                    if (kpUn.octave < minLevel)
                        continue;
                    if (maxLevel >= 0)
                        if (kpUn.octave > maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x - x;
                const float disty = kpUn.pt.y - y;

                if (fabs(distx) < r && fabs(disty) < r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

size_t Frame::GetCloestFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel, const int maxLevel) const
{
    size_t vIndices = N + 1;

    const int nMinCellX = max(0, (int)floor((x - mnMinX - r) * mfGridElementWidthInv));
    if (nMinCellX >= FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS - 1, (int)ceil((x - mnMinX + r) * mfGridElementWidthInv));
    if (nMaxCellX < 0)
        return vIndices;

    const int nMinCellY = max(0, (int)floor((y - mnMinY - r) * mfGridElementHeightInv));
    if (nMinCellY >= FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1, (int)ceil((y - mnMinY + r) * mfGridElementHeightInv));
    if (nMaxCellY < 0)
        return vIndices;

    const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);
    float min_dist = -1;
    for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
    {
        for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if (vCell.empty())
                continue;

            for (size_t j = 0, jend = vCell.size(); j < jend; j++)
            {
                if (KeysStatic.size() > 0 && KeysStatic[vCell[j]]) // this function is for dynamic point
                    continue;

                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if (bCheckLevels)
                {
                    if (kpUn.octave < minLevel)
                        continue;
                    if (maxLevel >= 0)
                        if (kpUn.octave > maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x - x;
                const float disty = kpUn.pt.y - y;

                if (fabs(distx) < r && fabs(disty) < r)
                {
                    float dist = distx * distx + disty * disty;
                    if (min_dist == -1 || dist < min_dist)
                    {
                        min_dist = dist;
                        vIndices = vCell[j];
                    }
                }
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
    posY = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
        return false;

    return true;
}

void Frame::ComputeBoW()
{
    if (mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
    }
}

void Frame::UndistortKeyPoints()
{
    if (mDistCoef.at<float>(0) == 0.0)
    {
        mvKeysUn = mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N, 2, CV_32F);
    for (int i = 0; i < N; i++)
    {
        mat.at<float>(i, 0) = mvKeys[i].pt.x;
        mat.at<float>(i, 1) = mvKeys[i].pt.y;
    }

    // Undistort points
    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
    mat = mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for (int i = 0; i < N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x = mat.at<float>(i, 0);
        kp.pt.y = mat.at<float>(i, 1);
        mvKeysUn[i] = kp;
    }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if (mDistCoef.at<float>(0) != 0.0)
    {
        cv::Mat mat(4, 2, CV_32F);
        mat.at<float>(0, 0) = 0.0;
        mat.at<float>(0, 1) = 0.0;
        mat.at<float>(1, 0) = imLeft.cols;
        mat.at<float>(1, 1) = 0.0;
        mat.at<float>(2, 0) = 0.0;
        mat.at<float>(2, 1) = imLeft.rows;
        mat.at<float>(3, 0) = imLeft.cols;
        mat.at<float>(3, 1) = imLeft.rows;

        // Undistort corners
        mat = mat.reshape(2);
        cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
        mat = mat.reshape(1);

        mnMinX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
        mnMaxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
        mnMinY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
        mnMaxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));
    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N, -1.0f);
    mvDepth = vector<float>(N, -1.0f);

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t>> vRowIndices(nRows, vector<size_t>());

    for (int i = 0; i < nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for (int iR = 0; iR < Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f * mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY + r);
        const int minr = floor(kpY - r);

        for (int yi = minr; yi <= maxr; yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = -3;
    const float maxD = mbf / minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int>> vDistIdx;
    vDistIdx.reserve(N);

    for (int iL = 0; iL < N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if (vCandidates.empty())
            continue;

        const float minU = uL - maxD;
        const float maxU = uL - minD;

        if (maxU < 0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for (size_t iC = 0; iC < vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if (kpR.octave < levelL - 1 || kpR.octave > levelL + 1)
                continue;

            const float &uR = kpR.pt.x;

            if (uR >= minU && uR <= maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL, dR);

                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if (bestDist < ORBmatcher::TH_HIGH)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x * scaleFactor);
            const float scaledvL = round(kpL.pt.y * scaleFactor);
            const float scaleduR0 = round(uR0 * scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduL - w, scaleduL + w + 1);
            IL.convertTo(IL, CV_32F);
            IL = IL - IL.at<float>(w, w) * cv::Mat::ones(IL.rows, IL.cols, CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2 * L + 1);

            const float iniu = scaleduR0 + L - w;
            const float endu = scaleduR0 + L + w + 1;
            if (iniu < 0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for (int incR = -L; incR <= +L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduR0 + incR - w, scaleduR0 + incR + w + 1);
                IR.convertTo(IR, CV_32F);
                IR = IR - IR.at<float>(w, w) * cv::Mat::ones(IR.rows, IR.cols, CV_32F);

                float dist = cv::norm(IL, IR, cv::NORM_L1);
                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestincR = incR;
                }

                vDists[L + incR] = dist;
            }

            if (bestincR == -L || bestincR == L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L + bestincR - 1];
            const float dist2 = vDists[L + bestincR];
            const float dist3 = vDists[L + bestincR + 1];

            const float deltaR = (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));

            if (deltaR < -1 || deltaR > 1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave] * ((float)scaleduR0 + (float)bestincR + deltaR);

            float disparity = (uL - bestuR);

            if (disparity >= 0 && disparity < maxD)
            {
                if (disparity <= 0)
                {
                    disparity = 0.01;
                    bestuR = uL - 0.01;
                }
                mvDepth[iL] = mbf / disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int, int>(bestDist, iL));
            }
        }
    }

    sort(vDistIdx.begin(), vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size() / 2].first;
    const float thDist = 1.5f * 1.4f * median;

    for (int i = vDistIdx.size() - 1; i >= 0; i--)
    {
        if (vDistIdx[i].first < thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second] = -1;
            mvDepth[vDistIdx[i].second] = -1;
        }
    }
}

void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N, -1);
    mvDepth = vector<float>(N, -1);

    for (int i = 0; i < N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v, u);

        if (d > 0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x - mbf / d;
        }
    }
}

cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if (z > 0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u - cx) * z * invfx;
        const float y = (v - cy) * z * invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);
        return mRwc * x3Dc + mOw;
    }
    else
        return cv::Mat();
}

cv::Mat Frame::UnprojectDepth(const int &i, float depth)
{
    const float z = depth;
    if (z > 0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u - cx) * z * invfx;
        const float y = (v - cy) * z * invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);
        return mRwc * x3Dc + mOw;
    }
    else
        return cv::Mat();
}

} // namespace ORB_SLAM2
