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
    
    cv::Mat_<float> design_matrix=(cv::Mat_<float>(8, 9) <<
    -conditioned_base[0][2]*conditioned_attach[0][0], -conditioned_base[0][2]*conditioned_attach[0][1], -conditioned_base[0][2]*conditioned_attach[0][2], 0, 0, 0, conditioned_base[0][0]*conditioned_attach[0][0], conditioned_base[0][0]*conditioned_attach[0][1], conditioned_base[0][0]*conditioned_attach[0][2],
    0, 0, 0, -conditioned_base[0][2]*conditioned_attach[0][0], -conditioned_base[0][2]*conditioned_attach[0][1], -conditioned_base[0][2]*conditioned_attach[0][2], conditioned_base[0][1]*conditioned_attach[0][0], conditioned_base[0][1]*conditioned_attach[0][1], conditioned_base[0][1]*conditioned_attach[0][2],
    -conditioned_base[1][2]*conditioned_attach[1][0], -conditioned_base[1][2]*conditioned_attach[1][1], -conditioned_base[1][2]*conditioned_attach[1][2], 0, 0, 0, conditioned_base[1][0]*conditioned_attach[1][0], conditioned_base[1][0]*conditioned_attach[1][1], conditioned_base[1][0]*conditioned_attach[1][2],
    0, 0, 0, -conditioned_base[1][2]*conditioned_attach[1][0], -conditioned_base[1][2]*conditioned_attach[1][1], -conditioned_base[1][2]*conditioned_attach[1][2], conditioned_base[1][1]*conditioned_attach[1][0], conditioned_base[1][1]*conditioned_attach[1][1], conditioned_base[1][1]*conditioned_attach[1][2],
    -conditioned_base[2][2]*conditioned_attach[2][0], -conditioned_base[2][2]*conditioned_attach[2][1], -conditioned_base[2][2]*conditioned_attach[2][2], 0, 0, 0, conditioned_base[2][0]*conditioned_attach[2][0], conditioned_base[2][0]*conditioned_attach[2][1], conditioned_base[2][0]*conditioned_attach[2][2],
    0, 0, 0, -conditioned_base[2][2]*conditioned_attach[2][0], -conditioned_base[2][2]*conditioned_attach[2][1], -conditioned_base[2][2]*conditioned_attach[2][2], conditioned_base[2][1]*conditioned_attach[2][0], conditioned_base[2][1]*conditioned_attach[2][1], conditioned_base[2][1]*conditioned_attach[2][2],
    -conditioned_base[3][2]*conditioned_attach[3][0], -conditioned_base[3][2]*conditioned_attach[3][1], -conditioned_base[3][2]*conditioned_attach[3][2], 0, 0, 0, conditioned_base[3][0]*conditioned_attach[3][0], conditioned_base[3][0]*conditioned_attach[3][1], conditioned_base[3][0]*conditioned_attach[3][2],
    0, 0, 0, -conditioned_base[3][2]*conditioned_attach[3][0], -conditioned_base[3][2]*conditioned_attach[3][1], -conditioned_base[3][2]*conditioned_attach[3][2], conditioned_base[3][1]*conditioned_attach[3][0], conditioned_base[3][1]*conditioned_attach[3][1], conditioned_base[3][1]*conditioned_attach[3][2]);

    std::cout << "design matrix" << design_matrix;
    return design_matrix;
    
    /**
    cv::Mat_<float> design_matrix=cv::Mat_<float>::zeros(int(conditioned_base.size()), 9);
    cv::Mat_<float> line1=cv::Mat_<float>::zeros(1, 9);
    cv::Mat_<float> line2=cv::Mat_<float>::zeros(1, 9);
    std::vector<cv::Mat_<float>> design_matrix_block;
    for(int i = 0;i<conditioned_base.size();i++){
        cv::Vec3f x_i = conditioned_base[i];
        cv::Vec3f x_f = conditioned_attach[i];
        std::vector<cv::Vec3f> line1_list={-conditioned_base[i][2]*conditioned_attach[i],
                                           cv::Vec3f(0.0,0.0,0.0),
                                           conditioned_base[i][0]*conditioned_attach[i]};
        std::vector<cv::Vec3f> line2_list={cv::Vec3f(0.0,0.0,0.0),
                                           -conditioned_base[i][2]*conditioned_attach[i],
                                           conditioned_base[i][1]*conditioned_attach[i]};


        cv::hconcat(line1_list,line1);
        cv::hconcat(line2_list,line2);
        //line1.t();
        //line2.t();
        design_matrix_block.push_back(line1);
        design_matrix_block.push_back(line2);
        cv::vconcat(line1,design_matrix);
        cv::vconcat(line2,design_matrix);

    }
    std::cout << design_matrix;
    std::cout << std::endl;
    //cv::vconcat(design_matrix_block,design_matrix);
    
    */ 


}


/**
 * @brief solve homogeneous equation system by usage of SVD
 * @param A the design matrix
 * @returns solution of the homogeneous equation system
 */
cv::Matx33f solve_dlt_homography2D(const cv::Mat_<float> &A)
{
    
    cv::SVD svd(A.t()*A, cv::SVD::FULL_UV);
    //std::cout << svd.w;
    //std::cout << svd.vt;
    //std::cout << svd.vt.row(8);

    cv::Mat h = cv::Mat(svd.vt.row(8)); // do we need to multiply by smallest svd.w here?
    //std::cout << h;

    cv::Mat H_cond = cv::Mat_<float>::zeros(3,3);

    H_cond.at<float>(0,0) = h.at<float>(0,1);
    H_cond.at<float>(0,1) = h.at<float>(0,2);
    H_cond.at<float>(0,2) = h.at<float>(0,3);
    H_cond.at<float>(1,0) = h.at<float>(0,4);
    H_cond.at<float>(1,1) = h.at<float>(0,5);
    H_cond.at<float>(1,2) = h.at<float>(0,6);
    H_cond.at<float>(2,0) = h.at<float>(0,7);
    H_cond.at<float>(2,1) = h.at<float>(0,8);
    H_cond.at<float>(2,2) = h.at<float>(0,9);

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
