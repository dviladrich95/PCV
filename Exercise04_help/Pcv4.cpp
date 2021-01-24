//============================================================================
// Name        : Pcv4.cpp
// Author      : Ronny Haensch, Andreas Ley
// Version     : 2.0
// Copyright   : -
// Description : Estimation of Fundamental Matrix
//============================================================================

#include "Pcv4.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/hal/interface.h>
#include <iostream>
#include <random>
#include <opencv2/features2d.hpp>

using std::cout;
using std::cin;
using std::endl;
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


            for (int i = 0; i< geomObjects.size(); i++){

                 cv::Vec3f point = geomObjects[i];


               cv::Matx33f results(point[0] * H(0,0) + point[1] * H(0,1) + point[2] * H(0,2),
                                   point[0] * H(1,0) + point[1] * H(1,1) + point[2] * H(1,2),
                                   point[0] * H(2,0) + point[1] * H(2,1) + point[2] * H(2,2));
                 cv::Vec3f resultsp(results(0,0), results(0,1), results(0,2));

                //cout << " Homography of points: "<< resultsp << endl;
                 result.push_back(resultsp);

            }
        } break;
        case GEOM_TYPE_LINE: {

            for (int i = 0; i< geomObjects.size() ; i++){
                cv::Vec3f line = geomObjects[i];

                cv::Matx33f Htra = H.t();
                cv::Matx33f Htrinv = Htra.inv();
                cv::Matx33f resultsl1(line[0] * Htrinv(0,0) + line[1] * Htrinv(0,1) + line[2] * Htrinv(0,2),
                                     line[0] * Htrinv(1,0) + line[1] * Htrinv(1,1) + line[2] * Htrinv(1,2),
                                     line[0] * Htrinv(2,0) + line[1] * Htrinv(2,1) + line[2]* Htrinv(2,2));

                cv::Vec3f resultsl(resultsl1(0,0), resultsl1(0,1), resultsl1(0,2));
                //cout << " Homography of Lines: "<< resultsp << endl;
                result.push_back(resultsl);
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
    std::vector<cv::Vec3f> porigin;
    float x_sum =0;
    float y_sum = 0;
    float x_sum1 =0;
    float y_sum1 = 0;
    float tx, ty,sx,sy;

    for(int i = 0;i< points2D.size(); i++){
        cv::Vec3f point = points2D[i];
        x_sum = x_sum + point[0];
        y_sum = y_sum + point[1];
    }
    tx = x_sum/points2D.size();
    ty = y_sum/points2D.size();

    for (int i = 0; i< points2D.size(); i++){
        float x_tr;
        float y_tr;
        cv::Vec3f point = points2D[i];
        x_tr = point[0] - tx;
        y_tr = point[1] - ty;
        cv::Vec3f pointsor(x_tr,y_tr,1);
        porigin.push_back(pointsor);
        x_sum1 = x_sum1 + abs(x_tr);
        y_sum1 = y_sum1 + abs(y_tr);
    }
    sx = x_sum1/points2D.size();
    sy = y_sum1/points2D.size();

    cv::Matx33f cond(1./sx, 0, -tx/sx, 0, 1./sy, -ty/sy, 0, 0, 1);

    return cond;

}


/**
 * @brief Define the design matrix as needed to compute fundamental matrix
 * @param p1 first set of points
 * @param p2 second set of points
 * @returns The design matrix to be computed
 */
cv::Mat_<float> getDesignMatrix_fundamental(const std::vector<cv::Vec3f>& p1_conditioned, const std::vector<cv::Vec3f>& p2_conditioned)
{
    cv::Mat A = cv::Mat::zeros(0,9,CV_32F);

    for (int i = 0; i <p1_conditioned.size();i++){
        cv::Vec3f p1 = p1_conditioned[i];
        cv::Vec3f p2 = p2_conditioned[i];

        float data[9] = {p1(0)*p2(0),p1(1)*p2(0),p1(2)*p2(0),p1(0)*p2(1),p1(1)*p2(1),p1(2)*p2(1),p1(0)*p2(2),p1(1)*p2(2),p1(2)*p2(2)};
        cv::Mat pd(1,9,CV_32F,&data);

        A.push_back(pd);
    }
    //cout<<A<<endl;
    return A;

}


/**
 * @brief Solve homogeneous equation system by usage of SVD
 * @param A The design matrix
 * @returns The estimated fundamental matrix
 */
cv::Matx33f solve_dlt_fundamental(const cv::Mat_<float>& A)
{
    cv::SVD svd(A,cv::SVD::FULL_UV);
    double min, max;
    cv::Point min_loc;
    cv::Point max_loc;
    cv::Mat w = svd.w;
    cv::Mat v = svd.vt;
    cv::minMaxLoc(w, &min, &max, &min_loc, &max_loc);
    cv::Mat F = cv::Mat::zeros(3,3,CV_32F);


    //cout<<svd.vt<<endl;
    v.row(min_loc.y + 1).colRange(0,3).copyTo(F.row(0));
    v.row(min_loc.y + 1).colRange(3,6).copyTo(F.row(1));
    v.row(min_loc.y + 1).colRange(6,9).copyTo(F.row(2));

    //cout<<F<<endl;

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
    double min, max;
    cv::Point min_loc;
    cv::Point max_loc;
    cv::Vec3f w = svd.w;
    cv::Mat u = svd.u;
    cv::Mat v = svd.vt;
    cv::minMaxLoc(w, &min, &max, &min_loc, &max_loc);

    w[min_loc.y] = 0;

    cv::Matx33f wm(w[0], 0, 0, 0, w[1], 0,0, 0,w[2]);

    //cout<<wm<<endl;
    cv::Mat Fs = u * wm *v;


    return Fs;
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
    cv::Matx33f T2t;
    transpose(T2, T2t);
    cv::Matx33f Fd =  T2t * F * T1;
    return Fd;
}


/**
 * @brief Compute the fundamental matrix
 * @param p1 first set of points
 * @param p2 second set of points
 * @return The estimated fundamental matrix
 */
cv::Matx33f getFundamentalMatrix(const std::vector<cv::Vec3f>& p1, const std::vector<cv::Vec3f>& p2)
{
    cv::Matx33f cp1 = getCondition2D(p1);
    cv::Matx33f cp2 = getCondition2D(p2);
    std::vector<cv::Vec3f> ap2D1 = applyH_2D(p1, cp1, GEOM_TYPE_POINT);
    std::vector<cv::Vec3f> ap2D2 = applyH_2D(p2, cp2, GEOM_TYPE_POINT);
    cv::Mat_<float> A = getDesignMatrix_fundamental(ap2D1,ap2D2);
    cv::Matx33f F_hat = solve_dlt_fundamental(A);
    cv::Matx33f Fc = forceSingularity(F_hat);
    cv::Matx33f F = decondition_fundamental(cp1,cp2,Fc);
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

    cv::Vec3d p1d = p1;
    cv::Vec3d p2d = p2;
    cv::Matx33d Fd = F;
    double d = cv::sampsonDistance(p1d,p2d,Fd);
    cout<<p1<<endl;
    return d;
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
    float counter = 0;
    for (int i = 0; i< p1.size(); i++){
        double d;
        cv::Vec3d po1 = p1[i];
        cv::Vec3d po2 = p2[i];
        d = cv::sampsonDistance(po1, po2, Fd);
        if (d < threshold){
            counter = counter + 1;
        }
    }
    return counter;
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
    //unsigned index = uniformDist(rng);
    //cout<<index<<endl;
    float topin = 0;
    cv::Matx33f Ff;
    for (int i = 0; i<numIterations; i++){
        std::vector<cv::Vec3f> p1s;
        std::vector<cv::Vec3f> p2s;
        for (int e =0; e< 8;e++){
            unsigned  ind = uniformDist(rng);
            p1s.push_back(p1[ind]);
            p2s.push_back(p2[ind]);
        }
        cv::Matx33d F = getFundamentalMatrix(p1s, p2s);
        float inly = countInliers(p1s, p2s, F,threshold);
        if (inly > topin){
            topin = inly;
            Ff = F;
        }
    }

    return Ff;
}




/**
 * @brief Draw points and corresponding epipolar lines into both images
 * @param img1 Structure containing first image
 * @param img2 Structure containing second image
 * @param p1 First point set (points in first image)
 * @param p2 First point set (points in second image)
 * @param F Fundamental matrix (mapping from point in img1 to lines in img2)
 */
void visualize(const cv::Mat& img1, const cv::Mat& img2, const std::vector<cv::Vec3f>& p1, const std::vector<cv::Vec3f>& p2, const cv::Matx33f& F) {

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

    const float ratio = 0.75f;

    for (const auto &pair : rawOrbMatches.matches_1_2) {

        // TO DO !!!
        // Skip those pairs that don't fulfill the ratio test or cross consistency check

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
    // TO DO !!!
}


}
