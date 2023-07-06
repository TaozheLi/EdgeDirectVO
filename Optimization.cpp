//
// Created by simmons on 7/5/23.
//
#include "Optimization.h"
#include "Pose.h"
#include "EdgeDirectVO.h"
#include "Settings.h"
#include <opencv4/opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv4/opencv2/opencv.hpp>

EdgeVO::Optimization::Optimization(){
//    MakeImageLocationVector(reference_edge.cols, reference_edge.rows);
    std::cout<<"call construction"<<std::endl;

}

void EdgeVO::Optimization::MakeImageLocationVector(const int w, const int h) {
    for(int y=0; y<h; y++){
        for(int x=0; x<w; x++) {
           mCurrentFrame2dPoints.row(0) << (x - mcx) * mfx_inv;
            mCurrentFrame2dPoints.row(1) << (y - mcy) * mfy_inv;
        }
    }
}

void EdgeVO::Optimization::DistanceOptimization(Eigen::Matrix<double, 6, Eigen::RowMajor> &poseupdate) {
    float lambda = 0.f;
    float error_last = EdgeVO::Settings::INF_F;
    float error = error_last;
//            std::cout<<EdgeVO::Settings::MAX_ITERATIONS_PER_PYRAMID[ lvl ]<<std::endl;
    for(int iter = 0; iter < MaxIterations; ++iter)
    {
        error_last = error;
        error = Loss(mPose.getPoseEigen());
        // Levenberg-Marquardt
        if( error < error_last)
        {
            // Update relative pose
            Eigen::Matrix<double, 6 , Eigen::RowMajor> del;
            ComputeDelta(lambda,  del);


            if( (del.segment<3>(0)).dot(del.segment<3>(0)) < EdgeVO::Settings::MIN_TRANSLATION_UPDATE &
                (del.segment<3>(3)).dot(del.segment<3>(3)) < EdgeVO::Settings::MIN_ROTATION_UPDATE    )
                break;
//                    std::cout<<del.rows();
            cv::Mat delMat = se3ExpEigen(del);
            mPose.updatePose( delMat );

            //Update lambda
            if(lambda <= EdgeVO::Settings::LAMBDA_MAX)
                lambda = EdgeVO::Settings::LAMBDA_MIN;
            else
                lambda *= EdgeVO::Settings::LAMBDA_UPDATE_FACTOR;
        }
        else
        {
            if(lambda == EdgeVO::Settings::LAMBDA_MIN)
                lambda = EdgeVO::Settings::LAMBDA_MAX;
            else
                lambda *= EdgeVO::Settings::LAMBDA_UPDATE_FACTOR;
        }
    }
}



float EdgeVO::Optimization::Loss(const Eigen::Matrix<double, 4, 4>&pose) {
    Eigen::Matrix<float,3,3> R = (pose.block<3,3>(0,0)).cast<float>() ;
    Eigen::Matrix<float,3,1> t = (pose.block<3,1>(0,3)).cast<float>() ;
    Warped2dInto3d();
    Warped3DInto2D(R, t);
    float loss = ComputeDistance();
    return loss;
}

void EdgeVO::Optimization::Warped2dInto3d() {
    mWarped3DPoints.row(0) = (mCurrentFrame2dPoints.row(0).array() - mcx ) * mfx_inv * mDepthEigen.array();
    mWarped3DPoints.row(1) = (mCurrentFrame2dPoints.row(1).array() - mcy ) * mfy_inv * mDepthEigen.array();
    mWarped3DPoints.row(2) = mDepthEigen.array();
}

void EdgeVO::Optimization::ComputeDelta(float &lambda, Eigen::Matrix<double, 6, Eigen::RowMajor> &del) {
    J.resize(mWarpedX.size(), Eigen::NoChange);
//    J.col(0) =  ;
//    J.col(1) = ;
//    J.col(2) = ;
//    J.col(3) = ;
//    J.col(4) = ;
//    J.col(5) = ;
}

void EdgeVO::Optimization::Warped3DInto2D(const Eigen::Matrix<float, 3, 3> &R, const Eigen::Matrix<float, 3, 1> &t) {
    mWarped3DPoints.resize(Eigen::NoChange, m3DPoints.cols());
    mWarped3DPoints = (R * m3DPoints + t.replicate(1, m3DPoints.rows()));
    mWarped2DPoints.row(0) = mfx * mWarped3DPoints.row(0).array() / mWarped3DPoints.row(2).array();
    mWarped2DPoints.row(1) = mfy * mWarped3DPoints.row(1).array() / mWarped3DPoints.row(2).array();
}

float EdgeVO::Optimization::ComputeDistance() {
    cv::Mat NormalizedEdgeImage(mReferenceEdgeImage.rows, mReferenceEdgeImage.cols, CV_32F);
    cv::normalize(mReferenceEdgeImage, NormalizedEdgeImage, 0, 1, 32);
    NormalizedEdgeImage = 1 - NormalizedEdgeImage;
    for(int index=0; index<mWarped2DPoints.cols(); index++) {
        int x = int(mWarped2DPoints.coeff(0, index));
        int y = int(mWarped2DPoints.coeff(1, index));
        NormalizedEdgeImage.at<uint8_t> (y, x) = NormalizedEdgeImage.at<uint8_t> (y, x) || 0;
    }
    cv::Mat DistanceImage;
    cv::distanceTransform(mReferenceEdgeImage, DistanceImage, cv::DIST_L2, 5, CV_32F);
    for(int index=0; index<mWarped2DPoints.cols(); index++){
        int x = int(mWarped2DPoints.coeff(0, index));
        int y = int(mWarped2DPoints.coeff(1, index));
        mDistances << DistanceImage.at<float>(y, x); // y denotes col, x denotes row
    }
    return mDistances.sum();
}

