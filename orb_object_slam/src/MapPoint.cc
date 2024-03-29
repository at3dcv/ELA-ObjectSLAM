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

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"

#include "Parameters.h"
#include "ORBmatcher.h"

#include <mutex>

// by me
#include "MapObject.h"
#include "Converter.h"

using namespace std;

namespace ORB_SLAM2
{

long unsigned int MapPoint::nNextId = 0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map *pMap) : mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
                                                                      mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
                                                                      mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
                                                                      mpReplaced(static_cast<MapPoint *>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3, 1, CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId = nNextId++;

    // me
    max_object_vote = 0;
    best_object = nullptr;
    mnGroundFittingForKF = 0;
    already_bundled = false;

    PosToObj = cv::Mat::zeros(3, 1, CV_32F);
}

MapPoint::MapPoint(const cv::Mat &Pos, Map *pMap, Frame *pFrame, const int &idxF) : mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
                                                                                    mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
                                                                                    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame *>(NULL)), mnVisible(1),
                                                                                    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector / cv::norm(mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor = pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist * levelScaleFactor;
    mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId = nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    if (is_dynamic && best_object != nullptr) // for dynamic pt, transfrom from object pose.
    {
        return Converter::toCvMat(best_object->GetWorldPos().pose.map(Converter::toVector3d(PosToObj)));
    }

    return mWorldPos.clone();
}

cv::Mat MapPoint::GetWorldPosBA()
{
    unique_lock<mutex> lock(mMutexPos);
    if (is_dynamic && best_object != nullptr)
    {
        return Converter::toCvMat(best_object->GetWorldPosBA().pose.map(Converter::toVector3d(PosToObj)));
    }

    return cv::Mat::zeros(0, 0, CV_32F); // should not happen, this function only for dynamic point
}

Eigen::Vector3f MapPoint::GetWorldPosVec()
{
    cv::Mat worldpose = GetWorldPos();
    return Eigen::Vector3f(worldpose.at<float>(0), worldpose.at<float>(1), worldpose.at<float>(2));
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

KeyFrame *MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame *pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if (mObservations.count(pKF)) // if the frame is already processed, skip it.
        return;
    mObservations[pKF] = idx;

    if (!use_dynamic_klt_features)
    {
        if (pKF->mvuRight[idx] >= 0)
            nObs += 2; // a stereo/RGBD camera projects to left/right camera. in g2o BA, both left/right is used. useful for constraint scale.
        else
            nObs++;
    }
    else
        nObs++;

    // NOTE points usually initially associated different objects, due to occlusions
    if (associate_point_with_object)
    {
        if (use_dynamic_klt_features && is_dynamic)
        {
            if (pKF->keypoint_associate_objectID_harris.size() > 0)
            {
                int frame_cubod_id = pKF->keypoint_associate_objectID_harris[idx];
                if (frame_cubod_id > -1)
                    AddObjectObservation(pKF->local_cuboids[frame_cubod_id]);
            }
        }
        else
        {
            if (pKF->keypoint_associate_objectID.size() > 0)
            {
                int frame_cubod_id = pKF->keypoint_associate_objectID[idx];

                // AC: reset keypoint_associate_objectID if no local_cuboids were proposed
                // AC: Otherwise, it will throw an error as it will access an empty vector
                if ((int)frame_cubod_id >= (int)pKF->local_cuboids.size())
                {
                    std::cout << "Reset keypoint_associate_objectID! " << frame_cubod_id << "/" << pKF->local_cuboids.size() << std::endl;
                    pKF->keypoint_associate_objectID[idx] = -1;
                    frame_cubod_id = -1;
                }
                if (frame_cubod_id > -1)
                {
                    // AC: Used for debugging...
                    // std::cout << "Found keypoint objectID association: " << frame_cubod_id << std::endl;
                    // std::cout << "Cuboid size: " << pKF->local_cuboids.size() << std::endl;
                    AddObjectObservation(pKF->local_cuboids[frame_cubod_id]);
                }
            }
        }
    }
}

void MapPoint::EraseObservation(KeyFrame *pKF)
{
    bool bBad = false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if (mObservations.count(pKF))
        {
            int idx = mObservations[pKF];
            if (pKF->mvuRight[idx] >= 0)
                nObs -= 2;
            else
                nObs--;

            mObservations.erase(pKF);

            if (mpRefKF == pKF)
                mpRefKF = mObservations.begin()->first; // is this sorted??????

            // If only 2 observations or less, discard point
            if (nObs <= 2)
                bBad = true;
            //TODO remove object observation...
        }
    }

    if (bBad)
        SetBadFlag();
}

map<KeyFrame *, size_t> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::AddObjectObservation(MapObject *obj)
{
    unique_lock<mutex> lock(mMutexObject);

    if (obj->already_associated)
    {
        MapObject *objlandmark = obj->associated_landmark;

        if (MapObjObservations.count(objlandmark))
            MapObjObservations[objlandmark]++;
        else
            MapObjObservations[objlandmark] = 1;

        // update best object, no need to call FindBestObject()
        if (MapObjObservations[objlandmark] > max_object_vote)
        {
            if ((best_object != nullptr) && (best_object != objlandmark))
                best_object->EraseUniqueMapPoint(this, max_object_vote);

            best_object = objlandmark;

            max_object_vote = MapObjObservations[objlandmark];
            best_object->AddUniqueMapPoint(this, max_object_vote); // even being added before, still increase observation num
        }
    }
    else
    {
        LocalObjObservations.insert(obj);
        obj->AddPotentialMapPoint(this); // for frame local object, still add point observation. will be useful when local object changes to landmark.
    }
}

void MapPoint::EraseObjectObservation(MapObject *obj)
{
    unique_lock<mutex> lock(mMutexObject);
    if (MapObjObservations.count(obj))
    {
        MapObjObservations[obj]--;
        FindBestObject();
    }
}

void MapPoint::FindBestObject()
{
    // for loop to update best beloned object, only for object landmarks, frame local object is not considered.
    best_object = nullptr;
    max_object_vote = 0;
    for (map<MapObject *, int>::iterator mit = MapObjObservations.begin(), mend = MapObjObservations.end(); mit != mend; mit++)
        if (mit->second > max_object_vote)
        {
            max_object_vote = mit->second;
            best_object = mit->first;
        }
}

int MapPoint::GetBelongedObject(MapObject *&obj)
{
    unique_lock<mutex> lock(mMutexObject);
    obj = best_object;
    return max_object_vote;
}

MapObject *MapPoint::GetBelongedObject()
{
    unique_lock<mutex> lock(mMutexObject);
    return best_object;
}

void MapPoint::SetBadFlag()
{
    map<KeyFrame *, size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad = true;
        obs = mObservations;
        mObservations.clear(); // remove observations from point side
    }
    for (map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++)
    {
        KeyFrame *pKF = mit->first;
        if (use_dynamic_klt_features && is_dynamic)
            pKF->EraseHarrisMapPointMatch(mit->second);
        else
            pKF->EraseMapPointMatch(mit->second); // remove observations from frame side
    }

    mpMap->EraseMapPoint(this);
}

MapPoint *MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(MapPoint *pMP)
{
    if (pMP->mnId == this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame *, size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs = mObservations;
        mObservations.clear();
        mbBad = true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for (map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame *pKF = mit->first;

        if (!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF, mit->second);
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible += n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound += n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound) / mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame *, size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if (mbBad)
            return;
        observations = mObservations;
    }

    if (observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
    {
        KeyFrame *pKF = mit->first;

        if (!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if (vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for (size_t i = 0; i < N; i++)
    {
        Distances[i][i] = 0;
        for (size_t j = i + 1; j < N; j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
            Distances[i][j] = distij;
            Distances[j][i] = distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for (size_t i = 0; i < N; i++)
    {
        vector<int> vDists(Distances[i], Distances[i] + N);
        sort(vDists.begin(), vDists.end());
        int median = vDists[0.5 * (N - 1)];

        if (median < BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if (mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame *, size_t> observations;
    KeyFrame *pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if (mbBad)
            return;
        observations = mObservations;
        pRefKF = mpRefKF;
        Pos = mWorldPos.clone();
    }

    if (observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
    int n = 0;
    for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
    {
        KeyFrame *pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali / cv::norm(normali);
        n++;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor = pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist * levelScaleFactor;
        mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];
        mNormalVector = normal / n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f * mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f * mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, const float &logScaleFactor)
{
    float ratio;
    {
        unique_lock<mutex> lock3(mMutexPos);
        ratio = mfMaxDistance / currentDist;
    }

    return ceil(log(ratio) / logScaleFactor);
}

} // namespace ORB_SLAM2
