//============================================================================
// Name        : Pcv4.cpp
// Author      : Ronny Haensch, Andreas Ley
// Version     : 2.0
// Copyright   : -
// Description : Estimation of Fundamental Matrix
//============================================================================

#include "Pcv4.h"

#include <random>
#include <opencv2/features2d.hpp>

using namespace cv;
using namespace std;


namespace pcv4 {
    
    /**
 * @brief Applies a 2D transformation to an array of points or lines
 * @param H Matrix representing the transformation
 * @param geomObjects Array of input objects, each in homogeneous coordinates
 * @param type The type of the geometric objects, point or line. All are the same type.
 * @returns Array of transformed objects.
 */
std::vector<cv::Vec3f> applyH_2D(const std::vector<cv::Vec3f>& geomObjects, const cv::Matx33f &H, GeometryType type)
{
    // TO DO !!!
    std::vector<cv::Vec3f> result;
    switch (type) {
        case GEOM_TYPE_POINT: {
            for(int i=0;i<geomObjects.size();i++){
                result.push_back(H*geomObjects[i]);
            }
        } break;
        case GEOM_TYPE_LINE: {
            for(int i=0;i<geomObjects.size();i++){
                result.push_back(H.inv().t()*geomObjects[i]);
            }
        } break;
        default:
            throw std::runtime_error("Unhandled geometry type!");
    }
    return result;

}


/**
 * @brief Get the conditioning matrix of given points
 * @param p The points as matrix
 * @returns The condition matrix
 */
cv::Matx33f getCondition2D(const std::vector<cv::Vec3f>& points2D)
{
    // TO DO !!!
    float num_points=points2D.size();
    cv::Vec2f sum_points(0.0,0.0);
    cv::Vec2f mean_point(0.0,0.0);
    double sum_sx=0.0;
    double sum_sy=0.0;
    double sx=0.0;
    double sy=0.0;

    for(const auto & point : points2D){
        cv::Vec2f point_eucl=cv::Vec2f(point[0]/point[2],point[1]/point[2]);
        sum_points=sum_points+point_eucl;
    }
    mean_point=sum_points/float(points2D.size());

    for(const auto & point : points2D){
        cv::Vec2f point_eucl=cv::Vec2f(point[0]/point[2],point[1]/point[2]);
        cv::Vec2f point_eucl_new=point_eucl-mean_point;

        sum_sx =sum_sx+cv::abs(point_eucl_new[0]);
        sum_sy=sum_sy+cv::abs(point_eucl_new[1]);
    }
    sx=sum_sx/float(points2D.size());
    sy=sum_sy/float(points2D.size());
    cv::Matx33f condition_matrix2D=cv::Matx33f(1.0/sx,0,-mean_point[0]/sx,
                                               0,1.0/sy,-mean_point[1]/sy,
                                               0,0,1);
    return condition_matrix2D;
}


/**
 * @brief Define the design matrix as needed to compute fundamental matrix
 * @param p1 first set of points
 * @param p2 second set of points
 * @returns The design matrix to be computed
 */
cv::Mat_<float> getDesignMatrix_fundamental(const std::vector<cv::Vec3f>& p1_conditioned, const std::vector<cv::Vec3f>& p2_conditioned)
{
    // TO DO !!!
    cv::Mat designMAt= cv::Mat::zeros(p1_conditioned.size(),9, CV_32FC1);

    for (int i=0; i<p1_conditioned.size(); i++){
        designMAt.at<float>(i,0)=(p1_conditioned[i][0]*p2_conditioned[i][0]);//x*x'
        designMAt.at<float>(i,1)=(p1_conditioned[i][1]*p2_conditioned[i][0]);//y*x'
        designMAt.at<float>(i,2)=(p1_conditioned[i][2]*p2_conditioned[i][0]);//w*x'
        designMAt.at<float>(i,3)=(p1_conditioned[i][0]*p2_conditioned[i][1]);//x*y'
        designMAt.at<float>(i,4)=(p1_conditioned[i][1]*p2_conditioned[i][1]);//y*y'
        designMAt.at<float>(i,5)=(p1_conditioned[i][2]*p2_conditioned[i][1]);//w*y'
        designMAt.at<float>(i,6)=(p1_conditioned[i][0]*p2_conditioned[i][2]);//x*w'
        designMAt.at<float>(i,7)=(p1_conditioned[i][1]*p2_conditioned[i][2]);//y*w'
        designMAt.at<float>(i,8)=(p1_conditioned[i][2]*p2_conditioned[i][2]);//w*w'

    }

    return designMAt;
}


/**
 * @brief Solve homogeneous equation system by usage of SVD
 * @param A The design matrix
 * @returns The estimated fundamental matrix
 */
cv::Matx33f solve_dlt_fundamental(const cv::Mat_<float>& A)
{
    // TO DO !!!
    cv::SVD svd(A,cv::SVD::FULL_UV);
    cv::Mat_<float> F = cv::Mat_<float>::zeros(3,3);
    //std::cout << svd.w << std::endl;
    //std::cout << svd.vt << std::endl;

    F.at<float>(0,0) = svd.vt.at<float>(8,0);
    F.at<float>(0,1) = svd.vt.at<float>(8,1);
    F.at<float>(0,2) = svd.vt.at<float>(8,2);
    F.at<float>(1,0) = svd.vt.at<float>(8,3);
    F.at<float>(1,1) = svd.vt.at<float>(8,4);
    F.at<float>(1,2) = svd.vt.at<float>(8,5);
    F.at<float>(2,0) = svd.vt.at<float>(8,6);
    F.at<float>(2,1) = svd.vt.at<float>(8,7);
    F.at<float>(2,2) = svd.vt.at<float>(8,8);

    return F;
}


