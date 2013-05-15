#include "RectifyDriver.h"

#include <PbMsgs/Image.h>



namespace hal
{

void CreateLookupTable(
        const calibu::CameraModelInterface& cam_from,
        const calibu::CameraModelInterface& cam_to,
        const Eigen::Matrix3d H_on,
        Eigen::Matrix<Eigen::Vector2f, Eigen::Dynamic, Eigen::Dynamic>& lookup_warp
        )
{
    for(size_t r = 0; r < cam_to.Height(); ++r) {
        for(size_t c = 0; c < cam_to.Width(); ++c) {
            const Eigen::Vector3d p_o = H_on * Eigen::Vector3d(c,r,1);
            
            // Remap
            Eigen::Vector2d p_warped = cam_from.Map(cam_to.Unmap(
                   calibu::Project<double>(p_o)
            ));
            
            // Clamp to valid image coords
            p_warped[0] = std::min(std::max(1.0, p_warped[0]), cam_to.Width() - 2.0 );
            p_warped[1] = std::min(std::max(1.0, p_warped[1]), cam_to.Height() - 2.0 );
            
//            p_warped[0] = c;
//            p_warped[1] = r;

            lookup_warp(r,c) = p_warped.cast<float>();
        }
    }
}

inline Sophus::SE3d CreateScanlineRectifiedLookupAndT_rl(
        const Sophus::SE3d T_rl,
        const calibu::CameraModelInterface& cam_left,
        const calibu::CameraModelInterface& cam_right,
        Eigen::Matrix<Eigen::Vector2f, Eigen::Dynamic, Eigen::Dynamic>& dlookup_left,
        Eigen::Matrix<Eigen::Vector2f, Eigen::Dynamic, Eigen::Dynamic>& dlookup_right
        )
{
    const Sophus::SO3d R_rl = T_rl.so3();
    const Sophus::SO3d R_lr = R_rl.inverse();
    const Eigen::Vector3d l_r = T_rl.translation();
    const Eigen::Vector3d r_l = - (R_lr * l_r);
    
    // Current up vector for each camera (in left FoR)
    const Eigen::Vector3d lup_l = Eigen::Vector3d(0,1,0);
    const Eigen::Vector3d rup_l = R_lr * Eigen::Vector3d(0,1,0);
    
    // Hypothetical fwd vector for each camera, perpendicular to baseline (in left FoR)
    const Eigen::Vector3d lfwd = lup_l.cross(r_l);
    const Eigen::Vector3d rfwd = rup_l.cross(r_l);
    
    // New fwd is average of left / right hypothetical baselines (also perpendicular to baseline)
    const Eigen::Vector3d new_fwd = (lfwd + rfwd).normalized();
    
    // Define new basis (in left FoR);
    const Eigen::Vector3d x = r_l.normalized();
    const Eigen::Vector3d z = -new_fwd;
    const Eigen::Vector3d y  = z.cross(x).normalized();
    
    // New orientation for both left and right cameras (expressed relative to original left)
    Eigen::Matrix3d mR_nl;
    mR_nl << x, y, z;
    
    // By definition, the right camera now lies exactly on the x-axis with the same orientation
    // as the left camera.
    const Sophus::SE3d T_nr_nl = Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-r_l.norm(),0,0) );
    
    // Homographies which should be applied to left and right images to scan-line rectify them
    const Eigen::Matrix3d Hl_nl = cam_left.K()  * mR_nl.transpose() * cam_left.Kinv();
    const Eigen::Matrix3d Hr_nr = cam_right.K() * (mR_nl * R_lr.matrix()).transpose() * cam_right.Kinv();
    
    const Eigen::Matrix3d Kl = cam_left.K();
    calibu::CameraModelT<calibu::Pinhole> cam_left_new( cam_left.Width(), cam_left.Height(), Eigen::Vector4d(Kl(0,0),Kl(1,1),Kl(0,2),Kl(1,2)) );

    const Eigen::Matrix3d Kr = cam_right.K();
    calibu::CameraModelT<calibu::Pinhole> cam_right_new( cam_right.Width(), cam_right.Height(), Eigen::Vector4d(Kr(0,0),Kr(1,1),Kr(0,2),Kr(1,2)) );

    std::cout << cam_left.K() << std::endl;
    std::cout << cam_left_new.K() << std::endl;
    std::cout << cam_right.K() << std::endl;
    std::cout << cam_right_new.K() << std::endl;
    
    
//    CreateLookupTable(cam_left, cam_left_new, Hl_nl, dlookup_left);
//    CreateLookupTable(cam_right, cam_right_new, Hr_nr, dlookup_right);

    CreateLookupTable(cam_left, cam_left_new, Eigen::Matrix3d::Identity(), dlookup_left);
    CreateLookupTable(cam_right, cam_right_new, Eigen::Matrix3d::Identity(), dlookup_right);
    
    return T_nr_nl;
}

void Remap(
        const Eigen::Matrix<Eigen::Vector2f, Eigen::Dynamic, Eigen::Dynamic>& lookup_warp,
        const pb::Image& in,
        pb::Image& out
        )
{
    for(int r=0; r<in.Height(); ++r) {
        for(int c=0; c<in.Width(); ++c) {
            const Eigen::Vector2f p = lookup_warp(r,c);
            out(r,c) = in(p(1),p(0));
//            out(r,c) = in(r,c);
        }
    }
}

void ToMatlabRemap(
        const Eigen::Matrix<Eigen::Vector2f, Eigen::Dynamic, Eigen::Dynamic>& lookup_warp,
        cv::Mat& mapx, cv::Mat& mapy
        )
{
    mapx = cv::Mat(lookup_warp.rows(), lookup_warp.cols(), CV_32F);
    mapy = cv::Mat(lookup_warp.rows(), lookup_warp.cols(), CV_32F);
    
    for(int r = 0; r < lookup_warp.rows(); ++r) {
        for(int c = 0; c < lookup_warp.cols(); ++c) {
            Eigen::Vector2f p = lookup_warp(r,c);
            mapx.at<float>(r,c) = c; //p[0];
            mapy.at<float>(r,c) = r; //p[1];
        }
    }
}


RectifyDriver::RectifyDriver(std::shared_ptr<CameraDriverInterface> input, const calibu::CameraRig& rig)
    : m_input(input)
{
    lookups.resize(rig.cameras.size());
    for(size_t i=0; i< rig.cameras.size(); ++i) {
        const calibu::CameraModel& cam = rig.cameras[i].camera;
        lookups[i] = Eigen::Matrix<Eigen::Vector2f, Eigen::Dynamic, Eigen::Dynamic>(cam.Height(), cam.Width());
    }
    
    if(rig.cameras.size() == 2) {
        // Stereo rectify

        // Generate lookup tables
        Sophus::SE3d T_nr_nl = CreateScanlineRectifiedLookupAndT_rl(
                rig.cameras[1].T_cw* rig.cameras[0].T_cw.inverse(),
                rig.cameras[0].camera,
                rig.cameras[1].camera,
                lookups[0], lookups[1]
                );
        
        for(int k=0; k < 2; ++k) {
            ToMatlabRemap(lookups[k], rmap[k][0], rmap[k][1]);
        }
    }
}

bool RectifyDriver::Capture( pb::CameraMsg& vImages )
{
    pb::CameraMsg vIn;
    
    const bool success = m_input->Capture( vIn );

    if(success) {
        vImages.Clear();
        
        pb::Image inimg[2] = { vIn.mutable_image(0), vIn.mutable_image(1)};
        
        for(int k=0; k < 2; ++k) {
            pb::ImageMsg* pimg = vImages.add_image();
            pimg->set_width(inimg[k].Width());
            pimg->set_height(inimg[k].Height());
            pimg->set_pitch(inimg[k].Pitch());
            pimg->set_type( (pb::Type)inimg[k].Type());
            pimg->set_format( (pb::Format)inimg[k].Format());
            pimg->mutable_data()->reserve(inimg[k].Width()*inimg[k].Height());
            
            pb::Image img = pb::Image(pimg);
            Remap(lookups[k], inimg[k], img );
//            cv::remap( (cv::Mat)img[k], rimg[k], rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
        }
        
//        for(int i=0; i<2; ++i) {
//            pb::ImageMsg* pImg = vImages.add_image();
//            pb::ReadCvMat(rimg[i], pImg);
//        }

    }
    
    return success;
}

std::string RectifyDriver::GetDeviceProperty(const std::string& sProperty)
{
    return m_input->GetDeviceProperty(sProperty);
}

}
