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
    cv::Vec2f sum_points(0.0,0.0);
    cv::Vec2f mean_point(0.0,0.0);
    float sum_sx=0.0;
    float sum_sy=0.0;
    float sx;
    float sy;

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
                                             0,1.0/sy,-mean_point[1]/sy,
                                             0,0,1);
    return condition_matrix;
}


/**
 * @brief define the design matrix as needed to compute 2D-homography
 * @param co// TO DO !!!nditioned_base first set of conditioned points x' --> x' = H * x
 * @param ca second set of conditioned points x --> x' = H * x
 * @returns the design matrix to be computed
 */
    cv::Mat_<float> getDesignMatrix_homography2D(const std::vector<cv::Vec3f> &cb, const std::vector<cv::Vec3f> &ca)
    {
        cv::Mat_<float> design_matrix=(cv::Mat_<float>(8, 9) <<
                -cb[0][2]*ca[0][0], -cb[0][2]*ca[0][1], -cb[0][2]*ca[0][2],0, 0, 0,cb[0][0]*ca[0][0], cb[0][0]*ca[0][1], cb[0][0]*ca[0][2],
                0, 0, 0,-cb[0][2]*ca[0][0], -cb[0][2]*ca[0][1], -cb[0][2]*ca[0][2],cb[0][1]*ca[0][0], cb[0][1]*ca[0][1], cb[0][1]*ca[0][2],
                -cb[1][2]*ca[1][0], -cb[1][2]*ca[1][1], -cb[1][2]*ca[1][2],0, 0, 0,cb[1][0]*ca[1][0], cb[1][0]*ca[1][1], cb[1][0]*ca[1][2],
                0, 0, 0,-cb[1][2]*ca[1][0], -cb[1][2]*ca[1][1], -cb[1][2]*ca[1][2],cb[1][1]*ca[1][0], cb[1][1]*ca[1][1], cb[1][1]*ca[1][2],
                -cb[2][2]*ca[2][0], -cb[2][2]*ca[2][1], -cb[2][2]*ca[2][2],0, 0, 0,cb[2][0]*ca[2][0], cb[2][0]*ca[2][1], cb[2][0]*ca[2][2],
                0, 0, 0, -cb[2][2]*ca[2][0], -cb[2][2]*ca[2][1], -cb[2][2]*ca[2][2],cb[2][1]*ca[2][0], cb[2][1]*ca[2][1], cb[2][1]*ca[2][2],
                -cb[3][2]*ca[3][0], -cb[3][2]*ca[3][1], -cb[3][2]*ca[3][2],0, 0, 0,cb[3][0]*ca[3][0], cb[3][0]*ca[3][1], cb[3][0]*ca[3][2],
                0, 0, 0, -cb[3][2]*ca[3][0], -cb[3][2]*ca[3][1], -cb[3][2]*ca[3][2],cb[3][1]*ca[3][0], cb[3][1]*ca[3][1], cb[3][1]*ca[3][2]);
        return design_matrix;
    }

    cv::Matx33f solve_dlt_homography2D(const cv::Mat_<float> &A)
    {
        cv::SVD svd(A,cv::SVD::FULL_UV);
        cv::Mat_<float> H_cond = cv::Mat_<float>::zeros(3,3);

        H_cond.at<float>(0,0) = svd.vt.at<float>(8,0);
        H_cond.at<float>(0,1) = svd.vt.at<float>(8,1);
        H_cond.at<float>(0,2) = svd.vt.at<float>(8,2);
        H_cond.at<float>(1,0) = svd.vt.at<float>(8,3);
        H_cond.at<float>(1,1) = svd.vt.at<float>(8,4);
        H_cond.at<float>(1,2) = svd.vt.at<float>(8,5);
        H_cond.at<float>(2,0) = svd.vt.at<float>(8,6);
        H_cond.at<float>(2,1) = svd.vt.at<float>(8,7);
        H_cond.at<float>(2,2) = svd.vt.at<float>(8,8);
        return H_cond;
    }

/**
 * @brief decondition a homography that was estimated from conditioned point clouds
 * @param T_base conditioning matrix T' of first set of points x'
 * @param T_attach conditioning matrix T of second set of points x
 * @param H conditioned homography that has to be un-conditioned (in-place)
 */
cv::Matx33f decondition_homography2D(const cv::Matx33f &T_base, const cv::Matx33f &T_attach, const cv::Matx33f &H)
{
    float sx= T_base(0,0);
    float sy= T_base(1,1);
    float tx= T_base(0,2);
    float ty= T_base(1,2);

    cv::Matx33f T_base_inv= cv::Matx33f(1/sx,0,-tx/sx,
                                        0,1/sy,-ty/sy,
                                        0,0,1);
    cv::Matx33f H_decon = T_base_inv * H * T_attach;
    return H_decon;
}


/**
 * @brief compute the homography
 * @param base first set of points x'
 * @param attach second set of points x
 * @returns homography H, so that x' = Hx
 */
cv::Matx33f homography2D(const std::vector<cv::Vec3f> &base, const std::vector<cv::Vec3f> &attach)
{
        cv::Matx33f base_cond = getCondition2D(base);

        cv::Matx33f attach_cond = getCondition2D(attach);

        std::vector<cv::Vec3f> c_baseMAT;
        std::vector<cv::Vec3f> c_attachMAT;
        c_baseMAT = applyH_2D(base, base_cond, GEOM_TYPE_POINT);
        c_attachMAT = applyH_2D(attach, attach_cond, GEOM_TYPE_POINT);

        cv::Mat_<float> DesignMat = getDesignMatrix_homography2D(c_baseMAT, c_attachMAT);
        cv::Matx33f dlt = solve_dlt_homography2D(DesignMat);
        cv::Matx33f Hom = decondition_homography2D(base_cond, attach_cond, dlt);
        return Hom;
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
                for(const auto & geomObject : geomObjects){
                    result.push_back(H*geomObject);
                }
            } break;
            case GEOM_TYPE_LINE: {
                for(const auto & geomObject : geomObjects){
                    result.push_back(H.inv().t()*geomObject);
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
