//============================================================================
// Name        : Pcv3.cpp
// Author      : Ronny Haensch
// Version     : 1.0
// Copyright   : -
// Description : Camera calibration
//============================================================================

#include "Pcv3.h"

namespace pcv3 {

/**
 * @brief get the conditioning matrix of given points
 * @param the points as matrix
 * @returns the condition matrix (already allocated)
 */
cv::Matx33f getCondition2D(const std::vector<cv::Vec3f> &points)
{
    float num_points=points.size();
    cv::Vec2f sum_points(0.0,0.0);
    cv::Vec2f mean_point(0.0,0.0);
    double sum_sx=0.0;
    double sum_sy=0.0;
    double sx=0.0;
    double sy=0.0;

    for(const auto & point : points){
        cv::Vec2f point_eucl=cv::Vec2f(point[0]/point[2],point[1]/point[2]);
        sum_points=sum_points+point_eucl;
    }
    mean_point=sum_points/float(points.size());

    for(const auto & point : points){
        cv::Vec2f point_eucl=cv::Vec2f(point[0]/point[2],point[1]/point[2]);
        cv::Vec2f point_eucl_new=point_eucl-mean_point;

        sum_sx =sum_sx+cv::abs(point_eucl_new[0]);
        sum_sy=sum_sy+cv::abs(point_eucl_new[1]);
    }
    sx=sum_sx/float(points.size());
    sy=sum_sy/float(points.size());
    cv::Matx33f condition_matrix2D=cv::Matx33f(1.0/sx,0,-mean_point[0]/sx,
                                             0,1.0/sy,-mean_point[1]/sy,
                                             0,0,1);
    return condition_matrix2D;
}

/**
 * @brief get the conditioning matrix of given points
 * @param the points as matrix
 * @returns the condition matrix (already allocated)
 */
cv::Matx44f getCondition3D(const std::vector<cv::Vec4f> &points)
{
    float num_points=points.size();
    cv::Vec3f sum_points(0.0,0.0,0.0);
    cv::Vec3f mean_point(0.0,0.0,0.0);
    double sum_sx=0.0;
    double sum_sy=0.0;
    double sum_sz=0.0;
    double sx=0.0;
    double sy=0.0;
    double sz=0.0;

    for(const auto & point : points){
        cv::Vec3f point_eucl=cv::Vec3f(point[0]/point[3],
                                       point[1]/point[3],
                                       point[2]/point[3]);
        sum_points=sum_points+point_eucl;
    }
    mean_point=sum_points/float(points.size());

    for(const auto & point : points){
        cv::Vec3f point_eucl=cv::Vec3f(point[0]/point[3],
                                       point[1]/point[3],
                                       point[2]/point[3]);
        cv::Vec3f point_eucl_new=point_eucl-mean_point;

        sum_sx =sum_sx+cv::abs(point_eucl_new[0]);
        sum_sy=sum_sy+cv::abs(point_eucl_new[1]);
        sum_sz=sum_sz+cv::abs(point_eucl_new[2]);
    }
    sx=sum_sx/float(points.size());
    sy=sum_sy/float(points.size());
    sz=sum_sz/float(points.size());
    cv::Matx44f condition_matrix3D=cv::Matx44f(1.0/sx,0,0,-mean_point[0]/sx,
                                             0,1.0/sy,0,-mean_point[1]/sy,
                                             0,0,1.0/sz,-mean_point[2]/sz,
                                             0,0,0,1);
    return condition_matrix3D;
}

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

    /******* Small std::vector cheat sheet ************************************/
    /*
     *   Number of elements in vector:                 a.size()
     *   Access i-th element (reading or writing):     a[i]
     *   Resize array:                                 a.resize(count);
     *   Append an element to an array:                a.push_back(element);
     *     \-> preallocate memory for e.g. push_back:  a.reserve(count);
     */
    /**************************************************************************/

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
 * @brief Applies a 3D transformation to an array of points
 * @param H Matrix representing the transformation
 * @param points Array of input points, each in homogeneous coordinates
 * @returns Array of transformed objects.
 */
std::vector<cv::Vec4f> applyH_3D_points(const std::vector<cv::Vec4f>& points, const cv::Matx44f &H)
{
    std::vector<cv::Vec4f> result;
    for(int i=0;i<points.size();i++){
        result.push_back(H*points[i]);
    }
    return result;
}



/**
 * @brief Define the design matrix as needed to compute projection matrix
 * @param points2D Set of 2D points within the image
 * @param points3D Set of 3D points at the object
 * @returns The design matrix to be computed
 */
cv::Mat_<float> getDesignMatrix_camera(const std::vector<cv::Vec3f>& points2D, const std::vector<cv::Vec4f>& points3D)
{
    // TO DO !!!
       cv::Mat designMAt= cv::Mat::zeros(2*points2D.size(),12, CV_32FC1);
for (int i=0; i<points2D.size(); i++){

    int a= i*2;
    //-W*X
    designMAt.at<float>(a,0)=(-points2D[i][2]*points3D[i][0]);//-w'*u
    designMAt.at<float>(a,1)=(-points2D[i][2]*points3D[i][1]);//-w'*v
    designMAt.at<float>(a,2)=(-points2D[i][2]*points3D[i][2]);//-w'*w
    designMAt.at<float>(a,3)=(-points2D[i][2]*points3D[i][3]);//-w*z
    //u'*u
    designMAt.at<float>(a,8)=points2D[i][0]*points3D[i][0];//u'*u
    designMAt.at<float>(a,9)=points2D[i][0]*points3D[i][1];//u'*v
    designMAt.at<float>(a,10)=points2D[i][0]*points3D[i][2];//u'*w
    designMAt.at<float>(a,11)=points2D[i][0]*points3D[i][3];//u*z
    //-W*X
    designMAt.at<float>(a+1,4)=(-points2D[i][2]*points3D[i][0]);//-w'*u
    designMAt.at<float>(a+1,5)=(-points2D[i][2]*points3D[i][1]);//-w'*v
    designMAt.at<float>(a+1,6)=(-points2D[i][2]*points3D[i][2]);
    designMAt.at<float>(a+1,7)=(-points2D[i][2]*points3D[i][3]);//-w'*w

    //v'*u
    designMAt.at<float>(a+1,8)=points2D[i][1]*points3D[i][0];//v'*u
    designMAt.at<float>(a+1,9)=points2D[i][1]*points3D[i][1];//v'*v
    designMAt.at<float>(a+1,10)=points2D[i][1]*points3D[i][2];//v'*w
    designMAt.at<float>(a+1,11)=points2D[i][1]*points3D[i][3];


  }

    return designMAt;
}


/**
 * @brief Solve homogeneous equation system by usage of SVD
 * @param A The design matrix
 * @returns The estimated projection matrix
 */
cv::Matx34f solve_dlt_camera(const cv::Mat_<float>& A)
{
    // TO DO !!!
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
    // TO DO !!!
    
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
    // TO DO !!!
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

    return cv::Matx34f::eye();
    return cv::Matx34f::eye();
}



/**
 * @brief Extract and prints information about interior and exterior orientation from camera
 * @param P The 3x4 projection matrix, only "input" to this function
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

    cv::Matx33f M_norm = M*lambda;

    cv::RQDecomp3x3(M_norm, K, R);

    //cv::Matx31f C = -M_norm.inv()*(P.col(3)*lambda);

    cv::SVD svd(P,cv::SVD::FULL_UV);
    cv::Mat_<float> C = cv::Mat_<float>::zeros(1,4);
    //std::cout << svd.w << std::endl;
    //std::cout << svd.vt << std::endl;

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
    info.skew = std::atan2(-R.val[1],R.val[0])*360/(2*M_PI);

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
    //info.cameraLocation(0) = ...;
    //info.cameraLocation(1) = ...;
    //info.cameraLocation(2) = ...;

}




}
