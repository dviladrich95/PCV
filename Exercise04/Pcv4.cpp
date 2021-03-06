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

    cv::Vec2f sum_points(0.0,0.0);
    cv::Vec2f mean_point(0.0,0.0);
    float sum_sx=0.0;
    float sum_sy=0.0;
    float sx=0.0;
    float sy=0.0;

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

    cv::Mat designMat= cv::Mat::zeros(p1_conditioned.size(),9, CV_32FC1);

    for (int i=0; i<p1_conditioned.size(); i++){
        designMat.at<float>(i,0)=(p1_conditioned[i][0]*p2_conditioned[i][0]);//x*x'
        designMat.at<float>(i,1)=(p1_conditioned[i][1]*p2_conditioned[i][0]);//y*x'
        designMat.at<float>(i,2)=(p1_conditioned[i][2]*p2_conditioned[i][0]);//w*x'
        designMat.at<float>(i,3)=(p1_conditioned[i][0]*p2_conditioned[i][1]);//x*y'
        designMat.at<float>(i,4)=(p1_conditioned[i][1]*p2_conditioned[i][1]);//y*y'
        designMat.at<float>(i,5)=(p1_conditioned[i][2]*p2_conditioned[i][1]);//w*y'
        designMat.at<float>(i,6)=(p1_conditioned[i][0]*p2_conditioned[i][2]);//x*w'
        designMat.at<float>(i,7)=(p1_conditioned[i][1]*p2_conditioned[i][2]);//y*w'
        designMat.at<float>(i,8)=(p1_conditioned[i][2]*p2_conditioned[i][2]);//w*w'

    }

    return designMat;
}


/**
 * @brief Solve homogeneous equation system by usage of SVD
 * @param A The design matrix
 * @returns The estimated fundamental matrix
 */
cv::Matx33f solve_dlt_fundamental(const cv::Mat_<float>& A)
{

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



/**
 * @brief Enforce rank of 2 on fundamental matrix
 * @param F The matrix to be changed
 * @return The modified fundamental matrix
 */
cv::Matx33f forceSingularity(const cv::Matx33f& F)
{
    cv::SVD svd(F,cv::SVD::FULL_UV);
    //std::cout << svd.u << std::endl;
    //std::cout << svd.w << std::endl;
    //std::cout << svd.vt << std::endl;

    cv::Matx33f u = svd.u;
    cv::Matx33f vt = svd.vt;
    cv::Matx33f diag = cv::Matx33f(svd.w.at<float>(0,0), 0, 0,
                                   0, svd.w.at<float>(0,1),0,
                                   0, 0, 0);

    cv::Matx33f F_forced = u * diag * vt;

    return F_forced;
}


/**
 * @brief Decondition a fundamental matrix that was estimated from conditioned points
 * @param T1 Conditioning matrix of set of 2D image points
 * @param T2 Conditioning matrix of set of 2D image points
 * @param F Conditioned fundamental matrix that has to be un-conditioned
 * @return Un-conditioned fundamental matrix
 */
cv::Matx33f decondition_fundamental(const cv::Matx33f& T1, const cv::Matx33f& T2, const cv::Matx33f& F)
{
    cv::Matx33f F_decond = T2.t() * F * T1;
    return F_decond;
}


/**
 * @brief Compute the fundamental matrix
 * @param p1 first set of points
 * @param p2 second set of points
 * @return The estimated fundamental matrix
 */
cv::Matx33f getFundamentalMatrix(const std::vector<cv::Vec3f>& p1, const std::vector<cv::Vec3f>& p2)
{

    cv::Matx33f p1_cond = getCondition2D(p1);
    cv::Matx33f p2_cond = getCondition2D(p2);

    std::vector<cv::Vec3f> c_p1;
    std::vector<cv::Vec3f> c_p2;

    c_p1 = applyH_2D(p1, p1_cond, GEOM_TYPE_POINT);
    c_p2 = applyH_2D(p2, p2_cond, GEOM_TYPE_POINT);

    cv::Mat_<float> DesignMat = getDesignMatrix_fundamental(c_p1, c_p2);

    cv::Matx33f dlt = solve_dlt_fundamental(DesignMat);

    cv::Matx33f force_dlt = forceSingularity(dlt);

    cv::Matx33f F = decondition_fundamental(p1_cond, p2_cond, force_dlt);
    return F;

}



/**
 * @brief Calculate geometric error of estimated fundamental matrix for a single point pair
 * @details Implement the "Sampson distance"
 * @param p1		first point
 * @param p2		second point
 * @param F		fundamental matrix
 * @returns geometric error
 */
float getError(const cv::Vec3f& p1, const cv::Vec3f& p2, const cv::Matx33f& F)
{
    cv::Matx33f Ft = cv::Matx33f(1, 0, 1,
                                   0, 1,0,
                                   0, 0, 1);
    cv::Vec<float, 1> num_vec = (p2.t() * Ft * p1);
    float num = pow(num_vec(0),2);
    float denom1 = std::pow((Ft*p1)(0),2);
    float denom2 = std::pow((Ft*p1)(1),2);
    float denom3 = std::pow((Ft.t()*p2)(0),2);
    float denom4 = std::pow((Ft.t()*p2)(1),2);
    float denom = denom1+denom2+denom3+denom4;
    float sampson = num/denom;

    return sampson;
}

/**
 * @brief Calculate geometric error of estimated fundamental matrix for a set of point pairs
 * @details Implement the mean "Sampson distance"
 * @param p1		first set of points
 * @param p2		second set of points
 * @param F		fundamental matrix
 * @returns geometric error
 */
float getError(const std::vector<cv::Vec3f>& p1, const std::vector<cv::Vec3f>& p2, const cv::Matx33f& F)
{
    double totd = 0;
    cv::Matx33d Fd = F;
    for (int i = 0; i< p1.size(); i++){
        double d;
        cv::Vec3d po1 = p1[i];
        cv::Vec3d po2 = p2[i];
        d = cv::sampsonDistance(po1, po2, Fd);
        totd = totd + d;
    }
    double md = totd/(p1.size());
    return md;

}

/**
 * @brief Count the number of inliers of an estimated fundamental matrix
 * @param p1		first set of points
 * @param p2		second set of points
 * @param F		fundamental matrix
 * @param threshold Maximal "Sampson distance" to still be counted as an inlier
 * @returns		Number of inliers
 */
unsigned countInliers(const std::vector<cv::Vec3f>& p1, const std::vector<cv::Vec3f>& p2, const cv::Matx33f& F, float threshold)
{
    cv::Matx33d Fd = F;
    int inliers = 0;
    for (int i=0; i<p1.size();i++){
        double sampson2;
        cv::Vec3d p1_d = p1[i];
        cv::Vec3d p2_d = p2[i];
        sampson2 = cv::sampsonDistance(p1_d, p2_d, Fd);
        if (sampson2<=threshold){
            inliers= inliers + 1;
        }
    }
    return inliers;
}

/**
 * @brief Estimate the fundamental matrix robustly using RANSAC
 * @details Use the number of inliers as the score
 * @param p1 first set of points
 * @param p2 second set of points
 * @param numIterations How many subsets are to be evaluated
 * @param threshold Maximal "Sampson distance" to still be counted as an inlier
 * @returns The fundamental matrix
 */
cv::Matx33f estimateFundamentalRANSAC(const std::vector<cv::Vec3f>& p1, const std::vector<cv::Vec3f>& p2, unsigned numIterations, float threshold)
{
    const unsigned subsetSize = 8;

    std::mt19937 rng;
    std::uniform_int_distribution<unsigned> uniformDist(0, p1.size()-1);
    // Draw a random point index with unsigned index = uniformDist(rng);
    int max_num_inliers = 0;
    cv::Matx33f F_best;
    for(int i=0; i<numIterations;i++){
        std::vector<cv::Vec3f> p1_sample;
        std::vector<cv::Vec3f> p2_sample;
        for( int j=0;j<subsetSize;j++){
            unsigned index = uniformDist(rng);
            p1_sample.push_back(p1[index]);
            p2_sample.push_back(p2[index]);
        }
        cv::Matx33f F = getFundamentalMatrix(p1_sample,p2_sample);
        int num_inliers = countInliers(p1_sample,p2_sample,F,threshold);
        if(num_inliers>max_num_inliers){
            max_num_inliers=num_inliers;
            F_best=F;
        }
        //std::cout << max_num_inliers << std::endl;
    }
    return F_best;
}




/**
 * @brief Draw points and corresponding epipolar lines into both images
 * @param img1 Structure containing first image
 * @param img2 Structure containing second image
 * @param p1 First point set (points in first image)
 * @param p2 First point set (points in second image)
 * @param F Fundamental matrix (mapping from point in img1 to lines in img2)
 */
void visualize(const cv::Mat& img1, const cv::Mat& img2, const std::vector<cv::Vec3f>& p1, const std::vector<cv::Vec3f>& p2, const cv::Matx33f& F)
{
    // make a copy to not draw into the original images and destroy them
    cv::Mat img1_copy = img1.clone();
    cv::Mat img2_copy = img2.clone();

    for (int i=0; i<p1.size(); i++ ){
        cv::Vec3f x = p1[i];
        cv::Vec3f line = F * x;
        double a = line[0];
        double b = line[1];
        double c = line[2];

        drawEpiLine(img2_copy, a, b, c);
    }
    for (int j=0; j<p2.size(); j++ ){
        //cv::Vec3f x_p = p1[j];
        cv::Vec3f x_p = p2[j];
        cv::Vec3f line_p = F.t() * x_p;
        double a1 = line_p[0];
        double b1 = line_p[1];
        double c1 = line_p[2];

        drawEpiLine(img1_copy, a1, b1, c1);
    }

    for (int a=0; a<p1.size();a++){
        cv::circle(img1_copy, cv::Point2f(p1[a][0]/p1[a][2], p1[a][1]/p1[a][2]), 2, cv::Scalar(0, 255, 0), 2);
        cv::circle(img2_copy, cv::Point2f(p2[a][0]/p2[a][2], p2[a][1]/p2[a][2]), 2, cv::Scalar(0, 255, 0), 2);
    }

    // show images
    cv::namedWindow( "Epilines img1", 0 );
    cv::resizeWindow("Epilines img1", 500, 500);
    cv::imshow("Epilines img1", img1_copy);
    cv::namedWindow( "Epilines img2", 0 );
    cv::resizeWindow("Epilines img2", 500, 500);
    cv::imshow("Epilines img2", img2_copy);

    cv::waitKey(0);
}



/**
 * @brief Filters the raw matches
 * @details Applies cross consistency check and ratio test (ratio of 0.75) and returns the point pairs that pass both.
 * @param rawOrbMatches Structure containing keypoints and raw matches obtained from comparing feature descriptors (see Helper.h)
 * @param p1 Points within the first image (returned in the array by this method)
 * @param p2 Points within the second image (returned in the array by this method)
 */
void filterMatches(const RawOrbMatches &rawOrbMatches, std::vector<cv::Vec3f>& p1, std::vector<cv::Vec3f>& p2)
{

/******* Small std::map cheat sheet ************************************

// This std::map stores pairs of ints and floats (key value pairs). Each float (value) can quickly be looked up with it's corresponding int (key).
std::map<int, float> exampleMap;

// Looking up an element:
int key = 5;
auto it = exampleMap.find(key);
if (it == exampleMap.end()) {
    // no entry with key 5 in the map
} else {
    float value = it->second;
    // do s.th. with the value
}

// Iteration over all elements:
for (const auto &pair : exampleMap) {
    int key = pair.first;
    float value = pair.second;
}

**************************************************************************/

    p1.clear();
    p2.clear();

    unsigned match_size12=rawOrbMatches.matches_1_2.size();
    unsigned match_size21=rawOrbMatches.matches_2_1.size();
    cv::Mat match_img = cv::Mat(match_size12,match_size21, CV_8UC3, cv::Scalar(0, 0, 0));



    const float ratio = 0.4f;
    for (const auto &pair : rawOrbMatches.matches_1_2) {


        // Skip those pairs that don't fulfill the ratio test
        float pair_ratio= pair.second.closestDistance / pair.second.secondClosestDistance;

        if (pair_ratio>ratio) {continue;}

        // or cross consistency check
        unsigned match_12_ind=pair.second.closest;
        auto match_2 = rawOrbMatches.matches_2_1.find(match_12_ind);
        if (match_2 == rawOrbMatches.matches_2_1.end()) {
            // no entry in the map
            continue;
        } else {
            unsigned match_21_ind = match_2->second.closest;
            if (match_21_ind != pair.first) {continue;}

        }

        p1.push_back(rawOrbMatches.keypoints1[pair.first]);
        p2.push_back(rawOrbMatches.keypoints2[pair.second.closest]);
    }
}

/**
 * @brief Computes matches automatically.
 * @details Points will be in homogeneous coordinates.
 * @param img1 The first image
 * @param img2 The second image
 * @param p1 Points within the first image (returned in the array by this method)
 * @param p2 Points within the second image (returned in the array by this method)
 */
void getPointsAutomatic(const cv::Mat &img1, const cv::Mat &img2, std::vector<cv::Vec3f>& p1, std::vector<cv::Vec3f>& p2)
{
    RawOrbMatches rawOrbMatches = extractRawOrbMatches(img1,img2);
    filterMatches(rawOrbMatches,p1,p2);
}


}
