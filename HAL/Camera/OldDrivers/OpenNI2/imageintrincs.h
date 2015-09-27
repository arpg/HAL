#ifndef IMAGEINTRINCS
#define IMAGEINTRINCS

#include <Eigen/Eigen>

struct ImageIntrinsics
{
    //////////////////////////////////////////////////////
    // Constructors
    //////////////////////////////////////////////////////
    inline
    void init(double dfu, double dfv, double du0, double dv0)
    {
        fu_ = dfu;
        fv_ = dfv;
        u0_ = du0;
        v0_ = dv0;
    }


    //////////////////////////////////////////////////////
    // Image projection
    //////////////////////////////////////////////////////

    inline
    Eigen::Vector2d Project(const Eigen::Vector3d P_c) const
    {
        Eigen::Vector2d val;
        val<< u0_ + fu_*P_c(0)/P_c(2),  v0_ + fv_*P_c(1)/P_c(2);
        return val;
    }

    inline
    Eigen::Vector2d Project(double x, double y, double z) const
    {
        Eigen::Vector2d val;
        val<<u0_ + fu_*x/z, v0_ + fv_*y/z;
        return val;
    }

    inline
    Eigen::Vector2d operator*(Eigen::Vector3d P_c) const
    {
        return Project(P_c);
    }

    //////////////////////////////////////////////////////
    // Image Unprojection
    //////////////////////////////////////////////////////

    inline
    Eigen::Vector3d Unproject(double u, double v) const
    {
        return Eigen::Vector3d((u-u0_)/fu_,(v-v0_)/fv_, 1);
    }

    inline
    Eigen::Vector3d Unproject(const Eigen::Vector2d p_c) const
    {
       Eigen::Vector3d val;
       val<<(p_c(0)-u0_)/fu_,(p_c(1)-v0_)/fv_, 1;
       return val;
    }

    inline
    Eigen::Vector3d Unproject(const Eigen::Vector2d p_c, double z) const
    {
        Eigen::Vector3d val;
        val<<z*(p_c(0)-u0_)/fu_,z*(p_c(1)-v0_)/fv_, z;
        return val;
    }

    inline
    Eigen::Vector3d Unproject(double u, double v, double z) const
    {

        Eigen::Vector3d val;
        val<<z*(u-u0_)/fu_,z*(v-v0_)/fv_, z;
        return val;
    }

    //////////////////////////////////////////////////////
    // Member variables
    //////////////////////////////////////////////////////

    double fu_;
    double fv_;
    double u0_;
    double v0_;
};


#endif // IMAGEINTRINCS

