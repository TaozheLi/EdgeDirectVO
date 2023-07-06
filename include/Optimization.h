//
// Created by simmons on 7/5/23.
//

#ifndef EDGEDIRECTVO_OPTIMIZATION_H
#define EDGEDIRECTVO_OPTIMIZATION_H
#include "KeyFrame.h"

#include <vector>
#include <opencv4/opencv2/opencv.hpp>
namespace EdgeVO{
class Optimization{
public:
     Optimization();
     void DistanceOptimization(Eigen::Matrix<double, 6, Eigen::RowMajor> &poseupdate);
     void MakeImageLocationVector(const int w, const int h);
     void Transfer(const Eigen::Matrix<float, 3, 3> &R, const Eigen::Matrix<float, 3, 1> &t);
     void Warped3DInto2D(const Eigen::Matrix<float, 3, 3> &R, const Eigen::Matrix<float, 3, 1> &t);
     void Warped2dInto3d();
     void ComputeDelta(float &lambda, Eigen::Matrix<double, 6, Eigen::RowMajor> &del);
     float Loss(const Eigen::Matrix<double, 4, 4> &pose);
     float ComputeDistance();

private:
    int MaxIterations;
    cv::Mat mReferenceEdgeImage;
    cv::Mat mTargetEdgeImage;
//    std::vector<float> m_warpedX;
//    std::vector<float> m_warpedY;
    Pose mPose;
    float mfx;
    float mfy;
    float mcx;
    float mcy;
    float mfx_inv;
    float mfy_inv;
    float mw;
    float mh;
    cv::Mat mDepth;
    Eigen::Matrix<float, Eigen::Dynamic, 1, Eigen::RowMajor> mCurrentFrameEdgeMask;
    Eigen::Matrix<float, Eigen::Dynamic, 1, Eigen::RowMajor> mReferenceFrameEdgeMask;
    Eigen::Matrix<float, Eigen::Dynamic, 1, Eigen::RowMajor> mFinalMask;
    Eigen::Matrix<float,  3, Eigen::Dynamic,Eigen::RowMajor> m3DPoints;
    Eigen::Matrix<float, 2, Eigen::Dynamic, Eigen::RowMajor> mCurrentFrame2dPoints;
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> mWarped3DPoints;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::RowMajor> mWarpedX;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::RowMajor> mWarpedY;
    Eigen::Matrix<float, 2, Eigen::Dynamic, Eigen::RowMajor> mWarped2DPoints;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::RowMajor> mDistances;
    Eigen::Matrix<float, 1, Eigen::Dynamic, Eigen::RowMajor> mDepthEigen;
    Eigen::Matrix<double, Eigen::Dynamic, 6, Eigen::RowMajor> J;
};

}
#endif //EDGEDIRECTVO_OPTIMIZATION_H
