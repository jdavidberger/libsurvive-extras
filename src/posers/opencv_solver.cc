#include "opencv_solver.h"

#ifndef USE_DOUBLE
#define FLT double
#define USE_DOUBLE
#endif

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <libsurvive/survive.h>
#include <libsurvive/poser.h>

struct sensor_data_t {
    uint32_t timecode = 0;
    double angle = 0;
    double length = 0;

    sensor_data_t() {}

    sensor_data_t(uint32_t timecode, double length, double angle) : timecode(timecode), angle(angle), length(length) {}
};

//LH, sensor_id, acode

std::vector<std::vector<cv::Point3f>> listOfobjectPoints[2];
std::vector<std::vector<cv::Point2f>> listOfimagePoints[2];

double scale = 1.;
double fov = 120. / 180. * M_PI;
double fov_2 = fov / 2.;
size_t fakeImageSize = fov * scale;
double f = 1; //scale * cos(fov_2) / sin(fov_2) * fov_2;
double cx = 0; // scale * fov_2,
double cy = cx;

cv::Mat cameraMatrix =
        (cv::Mat_<double>(3,3) <<
                      f, 0, cx,
0, f, cy,
0, 0, 1);

/*
void generateCameraMatrix(int lh) {
    cv::Mat dist, rvecs, tvecs;

    cv::calibrateCamera(listOfobjectPoints[lh], listOfimagePoints[lh], cv::Size(fakeImageSize, fakeImageSize), cameraMatrix[lh], dist, rvecs, tvecs, CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_FIX_PRINCIPAL_POINT);
    std::cerr << cameraMatrix[lh] << std::endl << std::endl;
    std::cerr << dist << std::endl << std::endl;
    std::cerr << rvecs<< std::endl << std::endl;
    std::cerr << tvecs<< std::endl << std::endl;

}
*/

int opencv_solver_poser_cb(SurviveObject* so, PoserData* pd) {
    switch(pd->pt) {
        case POSERDATA_FULL_SCENE: {
            auto pdfs = (PoserDataFullScene *)(pd);

            for(int lh = 0;lh < 2;lh++) {
                std::vector<cv::Point3f> cal_objectPoints;
                std::vector<cv::Point2f> cal_imagePoints;

                for (size_t i = 0; i < so->nr_locations; i++) {
                    auto& lengths = pdfs->lengths[i][lh];
                    auto& pt = pdfs->angles[i][lh];
                    if (lengths[0] < 0 || lengths[1] < 0) continue;

                    cal_imagePoints.emplace_back(tan(pt[0]) * scale, tan(pt[1]) * scale);
                    cal_objectPoints.emplace_back( so->sensor_locations[i * 3 + 0],
                                                   so->sensor_locations[i * 3 + 1],
                                                   so->sensor_locations[i * 3 + 2]);
                }

                if(cal_imagePoints.size() <= 4) {
                    auto ctx = so->ctx;
                    SV_INFO("Can't solve for only %lu points on lh %d\n", cal_imagePoints.size(), lh);
                    continue;
                }
                cv::Mat_<double> dist, rvec, tvec;
                std::vector<int> inliers;
/*
                cv::solvePnPRansac(cal_objectPoints, cal_imagePoints, cameraMatrix, dist, rvec, tvec,
                                   false, 100, 1., .99, inliers);
*/
                auto err = cv::solvePnP(cal_objectPoints, cal_imagePoints, cameraMatrix, dist, rvec, tvec, false, CV_EPNP);
                cv::Mat_<double> R;
                cv::Rodrigues(rvec, R); // R is 3x3

                std::cerr << cal_imagePoints << std::endl;
                std::cerr << cal_objectPoints << std::endl;
                std::cerr << cameraMatrix << std::endl;
                std::cerr << tvec << std::endl;
                std::cerr << R << std::endl;

                for(size_t i = 0;i < cal_objectPoints.size();i++) {
                    auto& obj = cal_objectPoints[i];
                    cv::Vec3d pt = { cal_imagePoints[i].x, cal_imagePoints[i].y, 1 };

                    cv::Vec3d v = cv::Mat(cameraMatrix * (R * cv::Mat_<double>(obj) + tvec));
                    std::cerr << cv::norm(pt - (v/v[2])) << std::endl;
                }

                R = R.t();  // rotation of inverse
                tvec = -R * tvec; // translation of inverse

                std::cerr << dist << std::endl;
                std::cerr << tvec << std::endl;
                std::cerr << R << std::endl;

                so->ctx->bsd[lh].PositionSet = 1;
                so->ctx->bsd[lh].Pose.Pos[0] = tvec[0][0];
                so->ctx->bsd[lh].Pose.Pos[1] = tvec[0][1];
                so->ctx->bsd[lh].Pose.Pos[2] = tvec[0][2];

                auto qw =
                        so->ctx->bsd[lh].Pose.Rot[0] = sqrt(1 + R[0][0] + R[1][1] + R[2][2]) / 2;
                so->ctx->bsd[lh].Pose.Rot[1] = (R[2][1] - R[1][2]) / 4. / qw;
                so->ctx->bsd[lh].Pose.Rot[2] = (R[0][2] - R[2][0]) / 4. / qw;
                so->ctx->bsd[lh].Pose.Rot[3] = (R[1][0] - R[0][1]) / 4. / qw;

            }
        }
        return 0;
    }
    return -1;
}
