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
    return cv::Mat_<float>(2*points2D.size(), 12);
}


/**
 * @brief Solve homogeneous equation system by usage of SVD
 * @param A The design matrix
 * @returns The estimated projection matrix
 */
cv::Matx34f solve_dlt_camera(const cv::Mat_<float>& A)
{
    // TO DO !!!
    return cv::Matx34f::eye();
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
    return P;
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
    // TO DO !!!

    // K = ...;
    // R = ...;
    
    /*
    // Principal distance or focal length
    info.principalDistance = ...;
    
    // Skew as an angle and in degrees
    info.skew = ...;
    
    // Aspect ratio of the pixels
    info.aspectRatio = ...;
    
    // Location of principal point in image (pixel) coordinates
    info.principalPoint(0) = ...;
    info.principalPoint(1) = ...;
    
    // Camera rotation angle 1/3
    info.omega = ...;
    
    // Camera rotation angle 2/3
    info.phi = ...;
    
    // Camera rotation angle 3/3
    info.kappa = ...;
    
    // 3D camera location in world coordinates
    info.cameraLocation(0) = ...;
    info.cameraLocation(1) = ...;
    info.cameraLocation(2) = ...;
    */
}




}
