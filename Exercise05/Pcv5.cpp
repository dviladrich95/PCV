//============================================================================
// Name        : Pcv5.cpp
// Author      : Andreas Ley
// Version     : 1.0
// Copyright   : -
// Description : Bundle Adjustment
//============================================================================

#include "Pcv5.h"

#include <random>
#include <opencv2/features2d.hpp>

using namespace cv;
using namespace std;

namespace pcv5 {


    

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
 * @brief Applies a 3D transformation to an array of points
 * @param H Matrix representing the transformation
 * @param points Array of input points, each in homogeneous coordinates
 * @returns Array of transformed objects.
 */
std::vector<cv::Vec4f> applyH_3D_points(const std::vector<cv::Vec4f>& geomObjects, const cv::Matx44f &H)
{
    std::vector<cv::Vec4f> result;
    for(const auto & geomObject : geomObjects){
        result.push_back(H*geomObject);
    }
    return result;
}
/**
 * @brief Get the conditioning matrix of given points
 * @param p The points as matrix
 * @returns The condition matrix
 */
cv::Matx44f getCondition3D(const std::vector<cv::Vec4f>& points3D)
{
    cv::Vec3f sum_points(0.0,0.0,0.0);
    cv::Vec3f mean_point(0.0,0.0,0.0);
    double sum_sx=0.0;
    double sum_sy=0.0;
    double sum_sz=0.0;
    double sx=0.0;
    double sy=0.0;
    double sz=0.0;

    for(const auto & point : points3D){
        cv::Vec3f point_eucl=cv::Vec3f(point[0]/point[3],
                                       point[1]/point[3],
                                       point[2]/point[3]);
        sum_points=sum_points+point_eucl;
    }
    mean_point=sum_points/float(points3D.size());

    for(const auto & point : points3D){
        cv::Vec3f point_eucl=cv::Vec3f(point[0]/point[3],
                                       point[1]/point[3],
                                       point[2]/point[3]);
        cv::Vec3f point_eucl_new=point_eucl-mean_point;

        sum_sx =sum_sx+cv::abs(point_eucl_new[0]);
        sum_sy=sum_sy+cv::abs(point_eucl_new[1]);
        sum_sz=sum_sz+cv::abs(point_eucl_new[2]);
    }
    sx=sum_sx/float(points3D.size());
    sy=sum_sy/float(points3D.size());
    sz=sum_sz/float(points3D.size());
    cv::Matx44f condition_matrix3D=cv::Matx44f(1.0/sx,0,0,-mean_point[0]/sx,
                                             0,1.0/sy,0,-mean_point[1]/sy,
                                             0,0,1.0/sz,-mean_point[2]/sz,
                                             0,0,0,1);
    return condition_matrix3D;
}






/**
 * @brief Define the design matrix as needed to compute projection matrix
 * @param points2D Set of 2D points within the image
 * @param points3D Set of 3D points at the object
 * @returns The design matrix to be computed
 */
cv::Mat_<float> getDesignMatrix_camera(const std::vector<cv::Vec3f>& points2D, const std::vector<cv::Vec4f>& points3D)
{
       cv::Mat designMat= cv::Mat::zeros(2*points2D.size(),12, CV_32FC1);
for (int i=0; i<points2D.size(); i++){

    int a= i*2;
    //-W*X
    designMat.at<float>(a,0)=(-points2D[i][2]*points3D[i][0]);//-w'*u
    designMat.at<float>(a,1)=(-points2D[i][2]*points3D[i][1]);//-w'*v
    designMat.at<float>(a,2)=(-points2D[i][2]*points3D[i][2]);//-w'*w
    designMat.at<float>(a,3)=(-points2D[i][2]*points3D[i][3]);//-w*z
    //u'*u
    designMat.at<float>(a,8)=points2D[i][0]*points3D[i][0];//u'*u
    designMat.at<float>(a,9)=points2D[i][0]*points3D[i][1];//u'*v
    designMat.at<float>(a,10)=points2D[i][0]*points3D[i][2];//u'*w
    designMat.at<float>(a,11)=points2D[i][0]*points3D[i][3];//u*z
    //-W*X
    designMat.at<float>(a+1,4)=(-points2D[i][2]*points3D[i][0]);//-w'*u
    designMat.at<float>(a+1,5)=(-points2D[i][2]*points3D[i][1]);//-w'*v
    designMat.at<float>(a+1,6)=(-points2D[i][2]*points3D[i][2]);
    designMat.at<float>(a+1,7)=(-points2D[i][2]*points3D[i][3]);//-w'*w

    //v'*u
    designMat.at<float>(a+1,8)=points2D[i][1]*points3D[i][0];//v'*u
    designMat.at<float>(a+1,9)=points2D[i][1]*points3D[i][1];//v'*v
    designMat.at<float>(a+1,10)=points2D[i][1]*points3D[i][2];//v'*w
    designMat.at<float>(a+1,11)=points2D[i][1]*points3D[i][3];
  }
    return designMat;
}

/**
 * @brief Solve homogeneous equation system by usage of SVD
 * @param A The design matrix
 * @returns The estimated projection matrix
 */
cv::Matx34f solve_dlt_camera(const cv::Mat_<float>& A)
{
        cv::SVD svd(A,cv::SVD::FULL_UV);
        cv::Mat_<float> P = cv::Mat_<float>::zeros(3,4);

        P.at<float>(0,0) = svd.vt.at<float>(11,0);
        P.at<float>(0,1) = svd.vt.at<float>(11,1);
        P.at<float>(0,2) = svd.vt.at<float>(11,2);
        P.at<float>(0,3) = svd.vt.at<float>(11,3);
        P.at<float>(1,0) = svd.vt.at<float>(11,4);
        P.at<float>(1,1) = svd.vt.at<float>(11,5);
        P.at<float>(1,2) = svd.vt.at<float>(11,6);
        P.at<float>(1,3) = svd.vt.at<float>(11,7);
        P.at<float>(2,0) = svd.vt.at<float>(11,8);
        P.at<float>(2,1) = svd.vt.at<float>(11,9);
        P.at<float>(2,2) = svd.vt.at<float>(11,10);
        P.at<float>(2,3) = svd.vt.at<float>(11,11);
        return P;
}

/**
 * @brief Decondition a projection matrix that was estimated from conditioned point clouds
 * @param T_2D Conditioning matrix of set of 2D image points
 * @param T_3D Conditioning matrix of set of 3D object points
 * @param P Conditioned projection matrix that has to be un-conditioned (in-place)
 */
cv::Matx34f decondition_camera(const cv::Matx33f& T_2D, const cv::Matx44f& T_3D, const cv::Matx34f& P)
{
    cv::Matx34f D= (T_2D.inv()) * P * T_3D;
    return D;
}

/**
 * @brief Estimate projection matrix
 * @param points2D Set of 2D points within the image
 * @param points3D Set of 3D points at the object
 * @returns The projection matrix to be computed
 */
cv::Matx34f calibrate(const std::vector<cv::Vec3f>& points2D, const std::vector<cv::Vec4f>& points3D)
{
    std::vector<cv::Vec3f>(c_P2D);
    std::vector<cv::Vec4f>(c_P3D);


    cv::Matx33f P2D = getCondition2D(points2D);
    cv::Matx44f P3D = getCondition3D(points3D);

    c_P2D = applyH_2D(points2D, P2D, GEOM_TYPE_POINT);
    c_P3D = applyH_3D_points(points3D,P3D);

    cv::Mat DMA = cv::Mat::zeros(2*points2D.size(),12, CV_32FC1);
    DMA = getDesignMatrix_camera(c_P2D,c_P3D);

    cv::Matx34f dlt = solve_dlt_camera(DMA);

    cv::Matx34f Hom = decondition_camera(P2D, P3D, dlt);

    return Hom;
}

/**
 * @brief Extract and prints information about interior and exterior orientation from camera
 * @param P The 3x4 projection matrix
 * @param K Matrix for returning the computed internal calibration
 * @param R Matrix for returning the computed rotation
 * @param info Structure for returning the interpretation such as principal distance
 */
void interprete(const cv::Matx34f &P, cv::Matx33f &K, cv::Matx33f &R, ProjectionMatrixInterpretation &info)
{
    cv::Matx33f M=cv::Matx33f(P.val[0], P.val[1], P.val[2],
                              P.val[4], P.val[5], P.val[6],
                              P.val[8], P.val[9], P.val[10]);

    float lambda = cv::determinant(M)/std::abs(cv::determinant(M))*1/cv::norm(M.row(2));


    cv::Matx33f d1=cv::Matx33f(-1, 0, 0,
                              0, 1, 0,
                              0, 0, 1);

    cv::Matx33f d2=cv::Matx33f(-1, 0, 0,
                               0, 1, 0,
                               0, 0, 1);

    cv::Matx33f d3=cv::Matx33f(-1, 0, 0,
                               0, 1, 0,
                               0, 0, 1);

    cv::Matx33f M_norm = M*lambda;
    cv::RQDecomp3x3(M_norm, K, R);

    if (K.val[0]<0){
        cv::Matx33f K= K.mul(d1);
        cv::Matx33f R= R.mul(d1);
    }
    if (K.val[4]<0){
        cv::Matx33f K= K.mul(d2);
        cv::Matx33f R= R.mul(d2);
    }
    if (K.val[8]<0){
        cv::Matx33f K= K.mul(d3);
        cv::Matx33f R= R.mul(d3);
    }

    //cv::Matx31f C = -M_norm.inv()*(P.col(3)*lambda);

    cv::SVD svd(P,cv::SVD::FULL_UV);
    cv::Mat_<float> C = cv::Mat_<float>::zeros(1,4);


    C.at<float>(0,0) = svd.vt.at<float>(3,0);
    C.at<float>(0,1) = svd.vt.at<float>(3,1);
    C.at<float>(0,2) = svd.vt.at<float>(3,2);
    C.at<float>(0,3) = svd.vt.at<float>(3,3);

    std::cout << C << std::endl;

    cv::Matx31f C_norm = cv::Matx31f(C[0][0]/C[0][3], C[0][1]/C[0][3], C[0][2]/C[0][3]);
    std::cout << C_norm << std::endl;



    // Principal distance or focal length
    info.principalDistance = K.val[0];

    // Skew as an angle and in degrees
    info.skew = std::atan2(K.val[0],-K.val[1])*360/(2*M_PI);

    // Aspect ratio of the pixels
    info.aspectRatio = K.val[4]/K.val[0];

    // Location of principal point in image (pixel) coordinates
    info.principalPoint(0) = K.val[2];
    info.principalPoint(1) = K.val[5];

    // Camera rotation angle 1/3
    info.omega = std::atan2(-R.val[7],R.val[8])*360/(2*M_PI);

    // Camera rotation angle 2/3
    info.phi = std::asin(R.val[6])*360/(2*M_PI);

    // Camera rotation angle 3/3
    info.kappa = std::atan2(-R.val[3],R.val[0])*360/(2*M_PI);

    // 3D camera location in world coordinates
    info.cameraLocation(0) = C_norm.val[0];
    info.cameraLocation(1) = C_norm.val[1];
    info.cameraLocation(2) = C_norm.val[2];

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
    // TO DO !!!
    cv::Matx33f F_decond = T2.t() * F * T1;
    return F_decond;
}


/**
 * @brief Compute the fundamental matrix
 * @param p1 first set of points
 * @param p2 second set of points
 * @returns	the estimated fundamental matrix
 */
cv::Matx33f getFundamentalMatrix(const std::vector<cv::Vec3f>& p1, const std::vector<cv::Vec3f>& p2)
{
    // TO DO !!!
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
 * @brief Computes the relative pose of two cameras given a list of point pairs and the camera's internal calibration.
 * @details The first camera is assumed to be in the origin, so only the external calibration of the second camera is computed. The point pairs are assumed to contain no outliers.
 * @param p1 Points in first image
 * @param p2 Points in second image
 * @param K Internal calibration matrix
 * @returns External calibration matrix of second camera
 */
cv::Matx44f computeCameraPose(const cv::Matx33f &K, const std::vector<cv::Vec3f>& p1, const std::vector<cv::Vec3f>& px2)
{

cv::Matx33f F=getFundamentalMatrix(p1,px2);

cv::Matx33f ES= K.t() *F *K;
    
cv::Matx33f R1,R2;
cv::Matx31f t;
cv::decomposeEssentialMat(ES,R1,R2,t);

cv::Mat_<float> EX = cv::Mat_<float>::zeros(4,4);
    EX.at<float>(0,0) = R1(0,0);
    EX.at<float>(0,1) = R1(0,1);
    EX.at<float>(0,2) = R1(0,2);
    EX.at<float>(0,3) = t(0,0);
    EX.at<float>(1,0) = R1(1,0);
    EX.at<float>(1,1) = R1(1,1);
    EX.at<float>(1,2) = R1(1,2);
    EX.at<float>(1,3) = t(0,1);
    EX.at<float>(2,0) = R1(2,0);
    EX.at<float>(2,1) = R1(2,1);
    EX.at<float>(2,2) = R1(2,2);
    EX.at<float>(2,3) = t(0,2);
    EX.at<float>(3,3) = 1;
    return EX;
/*
    Mat D, U, Vt;
    SVD::compute(E, D, U, Vt);

    if (determinant(U) < 0) U *= -1.;
    if (determinant(Vt) < 0) Vt *= -1.;

    Mat W = (Mat_<double>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 1);
    W.convertTo(W, E.type());

    Mat R1, R2, t;
    R1 = U * W * Vt;
    R2 = U * W.t() * Vt;
    t = U.col(2) * 1.0;
    */

}








/**
 * @brief Estimate the fundamental matrix robustly using RANSAC
 * @param p1 first set of points
 * @param p2 second set of points
 * @param numIterations How many subsets are to be evaluated
 * @returns The fundamental matrix
 */
cv::Matx34f estimateProjectionRANSAC(const std::vector<cv::Vec3f>& points2D, const std::vector<cv::Vec4f>& points3D, unsigned numIterations, float threshold)
{
    const unsigned subsetSize = 6;

    std::mt19937 rng;
    std::uniform_int_distribution<unsigned> uniformDist(0, points2D.size()-1);
    // Draw a random point index with unsigned index = uniformDist(rng);
    
    cv::Matx34f bestP;
    unsigned bestInliers = 0;
    
    std::vector<cv::Vec3f> points2D_subset;
    points2D_subset.resize(subsetSize);
    std::vector<cv::Vec4f> points3D_subset;
    points3D_subset.resize(subsetSize);
    for (unsigned iter = 0; iter < numIterations; iter++) {
        for (unsigned j = 0; j < subsetSize; j++) {
            unsigned index = uniformDist(rng);
            points2D_subset[j] = points2D[index];
            points3D_subset[j] = points3D[index];
        }
        
        cv::Matx34f P = calibrate(points2D_subset, points3D_subset);

        unsigned numInliers = 0;
        for (unsigned i = 0; i < points2D.size(); i++) {
            cv::Vec3f projected = P * points3D[i];
            if (projected(2) > 0.0f) // in front
                if ((std::abs(points2D[i](0) - projected(0)/projected(2)) < threshold) &&
                    (std::abs(points2D[i](1) - projected(1)/projected(2)) < threshold))
                    numInliers++;
        }

        if (numInliers > bestInliers) {
            bestInliers = numInliers;
            bestP = P;
        }
    }
    
    return bestP;
}


// triangulates given set of image points based on projection matrices
/*
P1	projection matrix of first image
P2	projection matrix of second image
x1	image point set of first image
x2	image point set of second image
return	triangulated object points
*/
cv::Vec4f linearTriangulation(const cv::Matx34f& P1, const cv::Matx34f& P2, const cv::Vec3f& x1, const cv::Vec3f& x2)
{
    // allocate memory for design matrix
    Mat_<float> A(4, 4);

    // create design matrix
    // first row	x1(0, i) * P1(2, :) - P1(0, :)
    A(0, 0) = x1(0) * P1(2, 0) - P1(0, 0);
    A(0, 1) = x1(0) * P1(2, 1) - P1(0, 1);
    A(0, 2) = x1(0) * P1(2, 2) - P1(0, 2);
    A(0, 3) = x1(0) * P1(2, 3) - P1(0, 3);
    // second row	x1(1, i) * P1(2, :) - P1(1, :)
    A(1, 0) = x1(1) * P1(2, 0) - P1(1, 0);
    A(1, 1) = x1(1) * P1(2, 1) - P1(1, 1);
    A(1, 2) = x1(1) * P1(2, 2) - P1(1, 2);
    A(1, 3) = x1(1) * P1(2, 3) - P1(1, 3);
    // third row	x2(0, i) * P2(3, :) - P2(0, :)
    A(2, 0) = x2(0) * P2(2, 0) - P2(0, 0);
    A(2, 1) = x2(0) * P2(2, 1) - P2(0, 1);
    A(2, 2) = x2(0) * P2(2, 2) - P2(0, 2);
    A(2, 3) = x2(0) * P2(2, 3) - P2(0, 3);
    // first row	x2(1, i) * P2(3, :) - P2(1, :)
    A(3, 0) = x2(1) * P2(2, 0) - P2(1, 0);
    A(3, 1) = x2(1) * P2(2, 1) - P2(1, 1);
    A(3, 2) = x2(1) * P2(2, 2) - P2(1, 2);
    A(3, 3) = x2(1) * P2(2, 3) - P2(1, 3);

    cv::SVD svd(A);
    Mat_<float> tmp = svd.vt.row(3).t();

    return cv::Vec4f(tmp(0), tmp(1), tmp(2), tmp(3));
}

std::vector<cv::Vec4f> linearTriangulation(const cv::Matx34f& P1, const cv::Matx34f& P2, const std::vector<cv::Vec3f>& x1, const std::vector<cv::Vec3f>& x2)
{
    std::vector<cv::Vec4f> result;
    result.resize(x1.size());
    for (unsigned i = 0; i < result.size(); i++)
        result[i] = linearTriangulation(P1, P2, x1[i], x2[i]);
    return result;
}



void BundleAdjustment::BAState::computeResiduals(float *residuals) const
{
    unsigned rIdx = 0;
    for (unsigned camIdx = 0; camIdx < m_cameras.size(); camIdx++) {
        const auto &calibState = m_internalCalibs[m_scene.cameras[camIdx].internalCalibIdx];
        const auto &cameraState = m_cameras[camIdx];
        
        // TO DO !!!
        // Compute 3x4 camera matrix (composition of internal and external calibration)
        // Internal calibration is calibState.K
        // External calibration is dropLastRow(cameraState.H)
        
        cv::Matx34f P ;// = ...
        
        for (const KeyPoint &kp : m_scene.cameras[camIdx].keypoints) {
            const auto &trackState = m_tracks[kp.trackIdx];
            // TO DO !!!
            // Using P, compute the homogeneous position of the track in the image (world space position is trackState.location)
            cv::Vec3f projection ;// = ...
            
            // TO DO !!!
            // Compute the euclidean position of the track
            
            // TO DO !!!
            // Compute the residuals: the difference between computed position and real position (kp.location(0) and kp.location(1))
            // Compute and store the (signed!) residual in x direction multiplied by kp.weight
            // residuals[rIdx++] = ...
            // Compute and store the (signed!) residual in y direction multiplied by kp.weight
            // residuals[rIdx++] = ...
        }
    }
}

void BundleAdjustment::BAState::computeJacobiMatrix(JacobiMatrix *dst) const
{
    BAJacobiMatrix &J = dynamic_cast<BAJacobiMatrix&>(*dst);
    
    unsigned rIdx = 0;
    for (unsigned camIdx = 0; camIdx < m_cameras.size(); camIdx++) {
        const auto &calibState = m_internalCalibs[m_scene.cameras[camIdx].internalCalibIdx];
        const auto &cameraState = m_cameras[camIdx];
        
        for (const KeyPoint &kp : m_scene.cameras[camIdx].keypoints) {
            const auto &trackState = m_tracks[kp.trackIdx];
            
            // calibState.K is the internal calbration
            // cameraState.H is the external calbration
            // trackState.location is the 3D location of the track in homogeneous coordinates

            // TO DO !!!
            // Compute the positions before and after the internal calibration (compare to slides).

            cv::Vec3f v ;// = ...
            cv::Vec3f u ;// = ...
            
            cv::Matx23f J_hom2eucl;
            // TO DO !!!
            // How do the euclidean image positions change when the homogeneous image positions change?
            /*
            J_hom2eucl(0, 0) = ...
            J_hom2eucl(0, 1) = ...
            J_hom2eucl(0, 2) = ...
            J_hom2eucl(1, 0) = ...
            J_hom2eucl(1, 1) = ...
            J_hom2eucl(1, 2) = ...
            */
            
            cv::Matx33f du_dDeltaK;
            /*
            // TO DO !!!
            // How do homogeneous image positions change when the internal calibration is changed (the 3 update parameters)?
            du_dDeltaK(0, 0) = ...
            du_dDeltaK(0, 1) = ...
            du_dDeltaK(0, 2) = ...
            du_dDeltaK(1, 0) = ...
            du_dDeltaK(1, 1) = ...
            du_dDeltaK(1, 2) = ...
            du_dDeltaK(2, 0) = ...
            du_dDeltaK(2, 1) = ...
            du_dDeltaK(2, 2) = ...
            */
            
            
            // TO DO !!!
            // Using the above (J_hom2eucl and du_dDeltaK), how do the euclidean image positions change when the internal calibration is changed (the 3 update parameters)?
            // Remember to include the weight of the keypoint (kp.weight)
            // J.m_rows[rIdx].J_internalCalib = 
            
            
            // TO DO !!!
            // How do the euclidean image positions change when the tracks are moving in eye space/camera space (the vector "v" in the slides)?
            cv::Matx<float, 2, 4> J_v2eucl; // works like cv::Matx24f but the latter was not typedef-ed
            
            
            //cv::Matx36f dv_dDeltaH;
            cv::Matx<float, 3, 6> dv_dDeltaH; // works like cv::Matx36f but the latter was not typedef-ed
            
            // TO DO !!!
            // How do tracks move in eye space (vector "v" in slides) when the parameters of the camera are changed?
            /*
            dv_dDeltaH(0, 0) = ...
            dv_dDeltaH(0, 1) = ...
            dv_dDeltaH(0, 2) = ...
            dv_dDeltaH(0, 3) = ...
            dv_dDeltaH(0, 4) = ...
            dv_dDeltaH(0, 5) = ...
            dv_dDeltaH(1, 0) = ...
            dv_dDeltaH(1, 1) = ...
            dv_dDeltaH(1, 2) = ...
            dv_dDeltaH(1, 3) = ...
            dv_dDeltaH(1, 4) = ...
            dv_dDeltaH(1, 5) = ...
            dv_dDeltaH(2, 0) = ...
            dv_dDeltaH(2, 1) = ...
            dv_dDeltaH(2, 2) = ...
            dv_dDeltaH(2, 3) = ...
            dv_dDeltaH(2, 4) = ...
            dv_dDeltaH(2, 5) = ...
            */
            
            // TO DO !!!
            // How do the euclidean image positions change when the external calibration is changed (the 6 update parameters)?
            // Remember to include the weight of the keypoint (kp.weight)
            // J.m_rows[rIdx].J_camera = 
            
            
            // TO DO !!!
            // How do the euclidean image positions change when the tracks are moving in world space (the x, y, z, and w before the external calibration)?
            // The multiplication operator "*" works as one would suspect. You can use dropLastRow(...) to drop the last row of a matrix.
            // cv::Matx<float, 2, 4> J_worldSpace2eucl =
            
            
            // TO DO !!!
            // How do the euclidean image positions change when the tracks are changed. 
            // This is the same as above, except it should also include the weight of the keypoint (kp.weight)
            // J.m_rows[rIdx].J_track = 
            
            rIdx++;
        }
    }
}

void BundleAdjustment::BAState::update(const float *update, State *dst) const
{
    BAState &state = dynamic_cast<BAState &>(*dst);
    state.m_internalCalibs.resize(m_internalCalibs.size());
    state.m_cameras.resize(m_cameras.size());
    state.m_tracks.resize(m_tracks.size());
    
    unsigned intCalibOffset = 0;
    for (unsigned i = 0; i < m_internalCalibs.size(); i++) {
        state.m_internalCalibs[i].K = m_internalCalibs[i].K;

        // TO DO !!!
        /*
        * Modify the new internal calibration
        * 
        * m_internalCalibs[i].K is the old matrix, state.m_internalCalibs[i].K is the new matrix.
        * 
        * update[intCalibOffset + i * NumUpdateParams::INTERNAL_CALIB + 0] is how much the focal length is supposed to change (scaled by the old focal length)
        * update[intCalibOffset + i * NumUpdateParams::INTERNAL_CALIB + 1] is how much the principal point is supposed to shift in x direction (scaled by the old x position of the principal point)
        * update[intCalibOffset + i * NumUpdateParams::INTERNAL_CALIB + 2] is how much the principal point is supposed to shift in y direction (scaled by the old y position of the principal point)
        */
    }
    unsigned cameraOffset = intCalibOffset + m_internalCalibs.size() * NumUpdateParams::INTERNAL_CALIB;
    for (unsigned i = 0; i < m_cameras.size(); i++) {
        // TO DO !!!
        /*
        * Compose the new matrix H
        * 
        * m_cameras[i].H is the old matrix, state.m_cameras[i].H is the new matrix.
        * 
        * update[cameraOffset + i * NumUpdateParams::CAMERA + 0] rotation increment around the camera X axis (not world X axis)
        * update[cameraOffset + i * NumUpdateParams::CAMERA + 1] rotation increment around the camera Y axis (not world Y axis)
        * update[cameraOffset + i * NumUpdateParams::CAMERA + 2] rotation increment around the camera Z axis (not world Z axis)
        * update[cameraOffset + i * NumUpdateParams::CAMERA + 3] translation increment along the camera X axis (not world X axis)
        * update[cameraOffset + i * NumUpdateParams::CAMERA + 4] translation increment along the camera Y axis (not world Y axis)
        * update[cameraOffset + i * NumUpdateParams::CAMERA + 5] translation increment along the camera Z axis (not world Z axis)
        * 
        * use rotationMatrixX(...), rotationMatrixY(...), rotationMatrixZ(...), and translationMatrix
        * 
        */

        //state.m_cameras[i].H = ...
    }
    unsigned trackOffset = cameraOffset + m_cameras.size() * NumUpdateParams::CAMERA;
    for (unsigned i = 0; i < m_tracks.size(); i++) {
        state.m_tracks[i].location = m_tracks[i].location;
        
        // TO DO !!!
        /*
        * Modify the new track location
        * 
        * m_tracks[i].location is the old location, state.m_tracks[i].location is the new location.
        * 
        * update[trackOffset + i * NumUpdateParams::TRACK + 0] increment of X
        * update[trackOffset + i * NumUpdateParams::TRACK + 1] increment of Y
        * update[trackOffset + i * NumUpdateParams::TRACK + 2] increment of Z
        * update[trackOffset + i * NumUpdateParams::TRACK + 3] increment of W
        */
        
        
        //state.m_tracks[i].location(0) += ...
        //state.m_tracks[i].location(1) += ...
        //state.m_tracks[i].location(2) += ...
        //state.m_tracks[i].location(3) += ...


        // Renormalization to length one
        float len = std::sqrt(state.m_tracks[i].location.dot(state.m_tracks[i].location));
        state.m_tracks[i].location *= 1.0f / len;
    }
}






/************************************************************************************************************/
/************************************************************************************************************/
/***************************                                     ********************************************/
/***************************    Nothing to do below this point   ********************************************/
/***************************                                     ********************************************/
/************************************************************************************************************/
/************************************************************************************************************/




BundleAdjustment::BAJacobiMatrix::BAJacobiMatrix(const Scene &scene)
{
    unsigned numResidualPairs = 0;
    for (const auto &camera : scene.cameras)
        numResidualPairs += camera.keypoints.size();
    
    m_rows.reserve(numResidualPairs);
    for (unsigned camIdx = 0; camIdx < scene.cameras.size(); camIdx++) {
        const auto &camera = scene.cameras[camIdx];
        for (unsigned kpIdx = 0; kpIdx < camera.keypoints.size(); kpIdx++) {
            m_rows.push_back({});
            m_rows.back().internalCalibIdx = camera.internalCalibIdx;
            m_rows.back().cameraIdx = camIdx;
            m_rows.back().keypointIdx = kpIdx;
            m_rows.back().trackIdx = camera.keypoints[kpIdx].trackIdx;
        }
    }
    
    m_internalCalibOffset = 0;
    m_cameraOffset = m_internalCalibOffset + scene.numInternalCalibs * NumUpdateParams::INTERNAL_CALIB;
    m_trackOffset = m_cameraOffset + scene.cameras.size() * NumUpdateParams::CAMERA;
    m_totalUpdateParams = m_trackOffset + scene.numTracks * NumUpdateParams::TRACK;
}

void BundleAdjustment::BAJacobiMatrix::multiply(float * __restrict dst, const float * __restrict src) const
{
    for (unsigned r = 0; r < m_rows.size(); r++) {
        float sumX = 0.0f;
        float sumY = 0.0f;
        for (unsigned i = 0; i < NumUpdateParams::INTERNAL_CALIB; i++) {
            sumX += src[m_internalCalibOffset + m_rows[r].internalCalibIdx * NumUpdateParams::INTERNAL_CALIB + i] * 
                        m_rows[r].J_internalCalib(0, i);
            sumY += src[m_internalCalibOffset + m_rows[r].internalCalibIdx * NumUpdateParams::INTERNAL_CALIB + i] * 
                        m_rows[r].J_internalCalib(1, i);
        }
        for (unsigned i = 0; i < NumUpdateParams::CAMERA; i++) {
            sumX += src[m_cameraOffset + m_rows[r].cameraIdx * NumUpdateParams::CAMERA + i] * 
                        m_rows[r].J_camera(0, i);
            sumY += src[m_cameraOffset + m_rows[r].cameraIdx * NumUpdateParams::CAMERA + i] * 
                        m_rows[r].J_camera(1, i);
        }
        for (unsigned i = 0; i < NumUpdateParams::TRACK; i++) {
            sumX += src[m_trackOffset + m_rows[r].trackIdx * NumUpdateParams::TRACK + i] * 
                        m_rows[r].J_track(0, i);
            sumY += src[m_trackOffset + m_rows[r].trackIdx * NumUpdateParams::TRACK + i] * 
                        m_rows[r].J_track(1, i);
        }
        dst[r*2+0] = sumX;
        dst[r*2+1] = sumY;
    }
}

void BundleAdjustment::BAJacobiMatrix::transposedMultiply(float * __restrict dst, const float * __restrict src) const
{
    memset(dst, 0, sizeof(float) * m_totalUpdateParams);
    // This is super ugly...
    for (unsigned r = 0; r < m_rows.size(); r++) {
        for (unsigned i = 0; i < NumUpdateParams::INTERNAL_CALIB; i++) {
            float elem = dst[m_internalCalibOffset + m_rows[r].internalCalibIdx * NumUpdateParams::INTERNAL_CALIB + i];
            elem += src[r*2+0] * m_rows[r].J_internalCalib(0, i);
            elem += src[r*2+1] * m_rows[r].J_internalCalib(1, i);
            dst[m_internalCalibOffset + m_rows[r].internalCalibIdx * NumUpdateParams::INTERNAL_CALIB + i] = elem;
        }
        
        for (unsigned i = 0; i < NumUpdateParams::CAMERA; i++) {
            float elem = dst[m_cameraOffset + m_rows[r].cameraIdx * NumUpdateParams::CAMERA + i];
            elem += src[r*2+0] * m_rows[r].J_camera(0, i);
            elem += src[r*2+1] * m_rows[r].J_camera(1, i);
            dst[m_cameraOffset + m_rows[r].cameraIdx * NumUpdateParams::CAMERA + i] = elem;
        }
        for (unsigned i = 0; i < NumUpdateParams::TRACK; i++) {
            float elem = dst[m_trackOffset + m_rows[r].trackIdx * NumUpdateParams::TRACK + i];
            elem += src[r*2+0] * m_rows[r].J_track(0, i);
            elem += src[r*2+1] * m_rows[r].J_track(1, i);
            dst[m_trackOffset + m_rows[r].trackIdx * NumUpdateParams::TRACK + i] = elem;
        }
    }
}

void BundleAdjustment::BAJacobiMatrix::computeDiagJtJ(float * __restrict dst) const
{
    memset(dst, 0, sizeof(float) * m_totalUpdateParams);
    // This is super ugly...
    for (unsigned r = 0; r < m_rows.size(); r++) {
        for (unsigned i = 0; i < NumUpdateParams::INTERNAL_CALIB; i++) {
            float elem = dst[m_internalCalibOffset + m_rows[r].internalCalibIdx * NumUpdateParams::INTERNAL_CALIB + i];
            elem += m_rows[r].J_internalCalib(0, i) * m_rows[r].J_internalCalib(0, i);
            elem += m_rows[r].J_internalCalib(1, i) * m_rows[r].J_internalCalib(1, i);
            dst[m_internalCalibOffset + m_rows[r].internalCalibIdx * NumUpdateParams::INTERNAL_CALIB + i] = elem;
        }
        for (unsigned i = 0; i < NumUpdateParams::CAMERA; i++) {
            float elem = dst[m_cameraOffset + m_rows[r].cameraIdx * NumUpdateParams::CAMERA + i];
            elem += m_rows[r].J_camera(0, i) * m_rows[r].J_camera(0, i);
            elem += m_rows[r].J_camera(1, i) * m_rows[r].J_camera(1, i);
            dst[m_cameraOffset + m_rows[r].cameraIdx * NumUpdateParams::CAMERA + i] = elem;
        }
        for (unsigned i = 0; i < NumUpdateParams::TRACK; i++) {
            float elem = dst[m_trackOffset + m_rows[r].trackIdx * NumUpdateParams::TRACK + i];
            elem += m_rows[r].J_track(0, i) * m_rows[r].J_track(0, i);
            elem += m_rows[r].J_track(1, i) * m_rows[r].J_track(1, i);
            dst[m_trackOffset + m_rows[r].trackIdx * NumUpdateParams::TRACK + i] = elem;
        }
    }
}



BundleAdjustment::BAState::BAState(const Scene &scene) : m_scene(scene)
{
    m_tracks.resize(m_scene.numTracks);
    m_internalCalibs.resize(m_scene.numInternalCalibs);
    m_cameras.resize(m_scene.cameras.size());
}

OptimizationProblem::State* BundleAdjustment::BAState::clone() const
{
    return new BAState(m_scene);
}


BundleAdjustment::BundleAdjustment(Scene &scene) : m_scene(scene)
{
    m_numResiduals = 0;
    for (const auto &camera : m_scene.cameras)
        m_numResiduals += camera.keypoints.size()*2;
    
    m_numUpdateParameters = 
                m_scene.numInternalCalibs * NumUpdateParams::INTERNAL_CALIB +
                m_scene.cameras.size() * NumUpdateParams::CAMERA +
                m_scene.numTracks * NumUpdateParams::TRACK;
}

OptimizationProblem::JacobiMatrix* BundleAdjustment::createJacobiMatrix() const
{
    return new BAJacobiMatrix(m_scene);
}


void BundleAdjustment::downweightOutlierKeypoints(BAState &state)
{
    std::vector<float> residuals;
    residuals.resize(m_numResiduals);
    state.computeResiduals(residuals.data());
    
    std::vector<float> distances;
    distances.resize(m_numResiduals/2);
    
    unsigned residualIdx = 0;
    for (auto &c : m_scene.cameras) {
        for (auto &kp : c.keypoints) {
            distances[residualIdx/2] = 
                std::sqrt(residuals[residualIdx+0]*residuals[residualIdx+0] + 
                          residuals[residualIdx+1]*residuals[residualIdx+1]);
            residualIdx+=2;
        }
    }

    std::vector<float> sortedDistances = distances;
    std::sort(sortedDistances.begin(), sortedDistances.end());
    
    std::cout << "min, max, median distances (weighted): " << sortedDistances.front() << " " << sortedDistances.back() << " " << sortedDistances[sortedDistances.size()/2] << std::endl;
    
    float thresh = sortedDistances[sortedDistances.size() * 2 / 3] * 2.0f;
    
    residualIdx = 0;
    for (auto &c : m_scene.cameras)
        for (auto &kp : c.keypoints) 
            if (distances[residualIdx++] > thresh) 
                kp.weight *= 0.5f;
}


Scene buildScene(const std::vector<std::string> &imagesFilenames)
{
    const float threshold = 20.0f;
    
    struct Image {
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        
        std::vector<std::vector<std::pair<unsigned, unsigned>>> matches;
    };
    
    std::vector<Image> allImages;
    allImages.resize(imagesFilenames.size());
    Ptr<ORB> orb = ORB::create();
    orb->setMaxFeatures(10000);
    for (unsigned i = 0; i < imagesFilenames.size(); i++) {
        std::cout << "Extracting keypoints from " << imagesFilenames[i] << std::endl;
        cv::Mat img = cv::imread(imagesFilenames[i].c_str());
        orb->detectAndCompute(img, cv::noArray(), allImages[i].keypoints, allImages[i].descriptors);
        allImages[i].matches.resize(allImages[i].keypoints.size());
    }
    
    Ptr<BFMatcher> matcher = BFMatcher::create(NORM_HAMMING);
    for (unsigned i = 0; i < allImages.size(); i++)
        for (unsigned j = i+1; j < allImages.size(); j++) {
            std::cout << "Matching " << imagesFilenames[i] << " against " << imagesFilenames[j] << std::endl;
            
            std::vector<std::vector<cv::DMatch>> matches;
            matcher->knnMatch(allImages[i].descriptors, allImages[j].descriptors, matches, 2);
            for (unsigned k = 0; k < matches.size(); ) {
                if (matches[k][0].distance > matches[k][1].distance * 0.75f) {
                    matches[k] = std::move(matches.back());
                    matches.pop_back();
                } else k++;
            }
            std::vector<cv::Vec3f> p1, p2;
            p1.resize(matches.size());
            p2.resize(matches.size());
            for (unsigned k = 0; k < matches.size(); k++) {
                p1[k] = cv::Vec3f(allImages[i].keypoints[matches[k][0].queryIdx].pt.x,
                                  allImages[i].keypoints[matches[k][0].queryIdx].pt.y,
                                  1.0f);
                p2[k] = cv::Vec3f(allImages[j].keypoints[matches[k][0].trainIdx].pt.x,
                                  allImages[j].keypoints[matches[k][0].trainIdx].pt.y,
                                  1.0f);
            }
            std::cout << "RANSACing " << imagesFilenames[i] << " against " << imagesFilenames[j] << std::endl;
            
            cv::Matx33f F = estimateFundamentalRANSAC(p1, p2, 1000, threshold);
            
            std::vector<std::pair<unsigned, unsigned>> inlierMatches;
            for (unsigned k = 0; k < matches.size(); k++) 
                if (getError(p1[k], p2[k], F) < threshold) 
                    inlierMatches.push_back({
                        matches[k][0].queryIdx,
                        matches[k][0].trainIdx
                    });
            const unsigned minMatches = 400;
                
            std::cout << "Found " << inlierMatches.size() << " valid matches!" << std::endl;
            if (inlierMatches.size() >= minMatches)
                for (const auto p : inlierMatches) {
                    allImages[i].matches[p.first].push_back({j, p.second});
                    allImages[j].matches[p.second].push_back({i, p.first});
                }
        }
    
    
    Scene scene;
    scene.numInternalCalibs = 1;
    scene.cameras.resize(imagesFilenames.size());
    for (auto &c : scene.cameras)
        c.internalCalibIdx = 0;
    scene.numTracks = 0;
    
    std::cout << "Finding tracks " << std::endl;
    {
        std::set<std::pair<unsigned, unsigned>> handledKeypoints;
        std::set<unsigned> imagesSpanned;
        std::vector<std::pair<unsigned, unsigned>> kpStack;
        std::vector<std::pair<unsigned, unsigned>> kpList;
        for (unsigned i = 0; i < allImages.size(); i++) {
            for (unsigned kp = 0; kp < allImages[i].keypoints.size(); kp++) {
                if (allImages[i].matches[kp].empty()) continue;
                if (handledKeypoints.find({i, kp}) != handledKeypoints.end()) continue;
                
                bool valid = true;
                
                kpStack.push_back({i, kp});
                while (!kpStack.empty()) {
                    auto kp = kpStack.back();
                    kpStack.pop_back();
                    
                    
                    if (imagesSpanned.find(kp.first) != imagesSpanned.end()) // appearing twice in one image -> invalid
                        valid = false;
                    
                    handledKeypoints.insert(kp);
                    kpList.push_back(kp);
                    imagesSpanned.insert(kp.first);
                    
                    for (const auto &matchedKp : allImages[kp.first].matches[kp.second])
                        if (handledKeypoints.find(matchedKp) == handledKeypoints.end()) 
                            kpStack.push_back(matchedKp);
                }
                
                if (valid) {
                    //std::cout << "Forming track from group of " << kpList.size() << " keypoints over " << imagesSpanned.size() << " images" << std::endl;
                    
                    for (const auto &kp : kpList) {
                        cv::Vec2f pixelPosition;
                        pixelPosition(0) = allImages[kp.first].keypoints[kp.second].pt.x;
                        pixelPosition(1) = allImages[kp.first].keypoints[kp.second].pt.y;
                        
                        unsigned trackIdx = scene.numTracks;
                        
                        scene.cameras[kp.first].keypoints.push_back({
                            pixelPosition,
                            trackIdx,
                            1.0f
                        });
                    }
                    
                    scene.numTracks++;
                } else {
                    //std::cout << "Dropping invalid group of " << kpList.size() << " keypoints over " << imagesSpanned.size() << " images" << std::endl;
                }
                kpList.clear();
                imagesSpanned.clear();
            }
        }
        std::cout << "Formed " << scene.numTracks << " tracks" << std::endl;
    }
    
    for (auto &c : scene.cameras)
        if (c.keypoints.size() < 100)
            std::cout << "Warning: One camera is connected with only " << c.keypoints.size() << " keypoints, this might be too unstable!" << std::endl;

    return scene;
}

void produceInitialState(const Scene &scene, const cv::Matx33f &initialInternalCalib, BundleAdjustment::BAState &state)
{
    const float threshold = 20.0f;
    
    state.m_internalCalibs[0].K = initialInternalCalib;
    
    std::set<unsigned> triangulatedPoints;
    
    const unsigned image1 = 0;
    const unsigned image2 = 1;
    // Find stereo pose of first two images
    {
        
        std::map<unsigned, cv::Vec2f> track2keypoint;
        for (const auto &kp : scene.cameras[image1].keypoints)
            track2keypoint[kp.trackIdx] = kp.location;
        
        std::vector<std::pair<cv::Vec2f, cv::Vec2f>> matches;
        std::vector<unsigned> matches2track;
        for (const auto &kp : scene.cameras[image2].keypoints) {
            auto it = track2keypoint.find(kp.trackIdx);
            if (it != track2keypoint.end()) {
                matches.push_back({it->second, kp.location});
                matches2track.push_back(kp.trackIdx);
            }
        }
        
        std::cout << "Initial pair has " << matches.size() << " matches" << std::endl;
        
        std::vector<cv::Vec3f> p1;
        p1.reserve(matches.size());
        std::vector<cv::Vec3f> p2;
        p2.reserve(matches.size());
        for (unsigned i = 0; i < matches.size(); i++) {
            p1.push_back(cv::Vec3f(matches[i].first(0), matches[i].first(1), 1.0f));
            p2.push_back(cv::Vec3f(matches[i].second(0), matches[i].second(1), 1.0f));
        }
        
        const cv::Matx33f &K = initialInternalCalib;
        state.m_cameras[image1].H = cv::Matx44f::eye();
        state.m_cameras[image2].H = computeCameraPose(K, p1, p2);
            
        std::vector<cv::Vec4f> X = linearTriangulation(K * cv::Matx34f::eye(), K * cv::Matx34f::eye() * state.m_cameras[image2].H, p1, p2);
        for (unsigned i = 0; i < X.size(); i++) {
            cv::Vec4f t = X[i];
            t /= std::sqrt(t.dot(t));
            state.m_tracks[matches2track[i]].location = t;
            triangulatedPoints.insert(matches2track[i]);
        }
    }
    

    for (unsigned c = 0; c < scene.cameras.size(); c++) {
        if (c == image1) continue;
        if (c == image2) continue;
        
        std::vector<KeyPoint> triangulatedKeypoints;
        for (const auto &kp : scene.cameras[c].keypoints) 
            if (triangulatedPoints.find(kp.trackIdx) != triangulatedPoints.end()) 
                triangulatedKeypoints.push_back(kp);

        if (triangulatedKeypoints.size() < 100)
            std::cout << "Warning: Camera " << c << " is only estimated from " << triangulatedKeypoints.size() << " keypoints" << std::endl;
        
        std::vector<cv::Vec3f> points2D;
        points2D.resize(triangulatedKeypoints.size());
        std::vector<cv::Vec4f> points3D;
        points3D.resize(triangulatedKeypoints.size());
        
        for (unsigned i = 0; i < triangulatedKeypoints.size(); i++) {
            points2D[i] = cv::Vec3f(
                        triangulatedKeypoints[i].location(0),
                        triangulatedKeypoints[i].location(1),
                        1.0f);
            points3D[i] = state.m_tracks[triangulatedKeypoints[i].trackIdx].location;
        }
        
        std::cout << "Estimating camera " << c << " from " << triangulatedKeypoints.size() << " keypoints" << std::endl;
        //cv::Mat P = calibrate(points2D, points3D);
        cv::Matx34f P = estimateProjectionRANSAC(points2D, points3D, 1000, threshold);
        cv::Matx33f K, R;
        ProjectionMatrixInterpretation info;
        interprete(P, K, R, info);
        
        state.m_cameras[c].H = cv::Matx44f::eye();
        for (unsigned i = 0; i < 3; i++)
            for (unsigned j = 0; j < 3; j++)
                state.m_cameras[c].H(i, j) = R(i, j);
            
        state.m_cameras[c].H = state.m_cameras[c].H * translationMatrix(-info.cameraLocation[0], -info.cameraLocation[1], -info.cameraLocation[2]);
    }
    // Triangulate remaining points
    for (unsigned c = 0; c < scene.cameras.size(); c++) {
        
        cv::Matx34f P1 = state.m_internalCalibs[scene.cameras[c].internalCalibIdx].K * cv::Matx34f::eye() * state.m_cameras[c].H;
            
        for (unsigned otherC = 0; otherC < c; otherC++) {
            cv::Matx34f P2 = state.m_internalCalibs[scene.cameras[otherC].internalCalibIdx].K * cv::Matx34f::eye() * state.m_cameras[otherC].H;
            for (const auto &kp : scene.cameras[c].keypoints) {
                if (triangulatedPoints.find(kp.trackIdx) != triangulatedPoints.end()) continue;
                
                for (const auto &otherKp : scene.cameras[otherC].keypoints) {
                    if (kp.trackIdx == otherKp.trackIdx) {
                        cv::Vec4f X = linearTriangulation(
                            P1, P2,
                            cv::Vec3f(kp.location(0), kp.location(1), 1.0f),
                            cv::Vec3f(otherKp.location(0), otherKp.location(1), 1.0f)
                        );
                        
                        X /= std::sqrt(X.dot(X));
                        state.m_tracks[kp.trackIdx].location = X;
                        
                        triangulatedPoints.insert(kp.trackIdx);
                    }
                }
            }
        }
    }
    if (triangulatedPoints.size() != state.m_tracks.size())
        std::cout << "Warning: Some tracks were not triangulated. This should not happen!" << std::endl;
}


}
