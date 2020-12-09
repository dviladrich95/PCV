//============================================================================
// Name        : Pcv2test.cpp
// Author      : Ronny Haensch
// Version     : 1.0
// Copyright   : -
// Description : 
//============================================================================


#include "Pcv2.h"
namespace pcv2 {


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
    cv::Matx33f condition_matrix=cv::Matx33f(1.0/sx,0,-mean_point[0]/sx,
                                             0,1.0/sx,-mean_point[1]/sy,
                                             0,0,1);
    return condition_matrix;
 }


/**
 * @brief define the design matrix as needed to compute 2D-homography
 * @param co// TO DO !!!nditioned_base first set of conditioned points x' --> x' = H * x
 * @param conditioned_attach second set of conditioned points x --> x' = H * x
 * @returns the design matrix to be computed
 */
cv::Mat_<float> getDesignMatrix_homography2D(const std::vector<cv::Vec3f> &conditioned_base, const std::vector<cv::Vec3f> &conditioned_attach)
{
    // TO DO !!!
    return cv::Mat_<float>::zeros(8, 9);
}


/**
 * @brief solve homogeneous equation system by usage of SVD
 * @param A the design matrix
 * @returns solution of the homogeneous equation system
 */
cv::Matx33f solve_dlt_homography2D(const cv::Mat_<float> &A)
{
    // TO DO !!!
    return cv::Matx33f::eye();
}


/**
 * @brief decondition a homography that was estimated from conditioned point clouds
 * @param T_base conditioning matrix T' of first set of points x'
 * @param T_attach conditioning matrix T of second set of points x
 * @param H conditioned homography that has to be un-conditioned (in-place)
 */
cv::Matx33f decondition_homography2D(const cv::Matx33f &T_base, const cv::Matx33f &T_attach, const cv::Matx33f &H) 
{
    // TO DO !!!
    return H;
}


/**
 * @brief compute the homography
 * @param base first set of points x'
 * @param attach second set of points x
 * @returns homography H, so that x' = Hx
 */
cv::Matx33f homography2D(const std::vector<cv::Vec3f> &base, const std::vector<cv::Vec3f> &attach)
{
    // TO DO !!!
    return cv::Matx33f::eye();
}



// Functions from exercise 1
// Reuse your solutions from the last exercise here

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
 * @brief Convert a 2D point from Euclidean to homogeneous coordinates
 * @param p The point to convert (in Euclidean coordinates)
 * @returns The same point in homogeneous coordinates
 */
cv::Vec3f eucl2hom_point_2D(const cv::Vec2f& p)
{
    return cv::Vec3f(p[0],p[1],1);
}


}