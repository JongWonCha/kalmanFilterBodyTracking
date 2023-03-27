#pragma comment(lib, "k4a.lib")
#pragma comment(lib, "k4abt.lib")
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <k4a/k4a.h>
#include <k4abt.h>
#include <Eigen/Dense>
#include <chrono>
using namespace std;
using namespace chrono;
using namespace Eigen;

#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }                                                                                                    \
/*
class skeleton //현 스켈레톤과 master 스켈레톤과의 차이
{
public:
    float px;
    float py;
    float pz;
    
    float rx = -680;
    float ry = 60;
    float rz = 12;
    
    float ox;
    float oy;
    float oz;
    float ow;
    Quat orien;

    int Count = 0;
};
*/
class kalmanFilter {
public:
    Matrix<double, 6, 1> xHat;           //추정값
    Matrix<double, 6, 6> covariance;     //오차공분산 행렬
    Matrix<double, 6, 6> A;              //상태전이 행렬
    Matrix<double, 3, 6> H;              //측정 행렬 - 추정값 행렬 중 무엇을 뽑아 쓸 것인가
    Matrix<double, 6, 6> processNoise;   //Q값 - 시스템 모델 오차
    Matrix<double, 3, 3> measureNoise;   //R값 - 기기의 측정 오차

    kalmanFilter(double proNoise, double meaNoise, double initX, double initY, double initZ ) {
        A = Matrix<double, 6, 3>::Identity();
        H = Matrix<double, 3, 6>::Zero();
        H(0, 0) = H(1, 1) = H(2, 2) = 1;
        xHat = Matrix<double, 6, 1>::Zero();
        covariance = Matrix<double, 6, 6>::Identity();
        processNoise = Matrix<double, 6, 6>::Identity() * proNoise;
        measureNoise = Matrix<double, 3, 3>::Identity() * meaNoise;
        xHat(0, 0) = initX;
        xHat(1, 0) = initY;
        xHat(2, 0) = initZ;
    }

    static kalmanFilter& getInstance(double proNoise, double meaNoise, double initX, double initY, double initZ) {
        static kalmanFilter instance(proNoise, meaNoise, initX, initY, initZ);
        return instance;
    }

    void predict(double dt) {
        A(0, 3) = A(1, 4) = A(2, 5) = dt;
        xHat = A * xHat;
        covariance = A * covariance * A.transpose() + processNoise;
    }

    void update(const Vector3f& z) {
        Vector3d y = (Vector3d)z - H * xHat;
        Matrix3d temp = H * covariance * H.transpose() * measureNoise;
        Matrix<double, 6, 3> kalmanGain = covariance * H.transpose() * temp.inverse();

        xHat = xHat * kalmanGain * y;
        covariance = (Matrix<double, 6, 6>::Identity() - kalmanGain * H) * covariance;
    }

    kalmanFilter(const kalmanFilter&) = delete;
    kalmanFilter operator=(const kalmanFilter&) = delete;
};


class Quat {
public:
    double w, x, y, z;

    Quat(double w = 1.0, double x = 0.0, double y = 0.0, double z = 0.0) : w(w), x(x), y(y), z(z) {}

    Quat operator*(const Quat& q) const {
        return Quat(w * q.w - x * q.x - y * q.y - z * q.z,
                    w * q.x + x * q.w + y * q.z - z * q.y,
                    w * q.y - x * q.z + y * q.w + z * q.x,
                    w * q.z + x * q.y - y * q.x + z * q.w);
    }

    Quat rotate(const Quat& q, const Quat& r) { return q * r * Quat(q.w, -q.x, -q.y, -q.z); }

    Quat inverse() const {
        double normSqure = w + w + x * x + y * y + z * z;
        double iNormSqure = 1.f / normSqure;
        return Quat(w * iNormSqure, -x * iNormSqure, -y * iNormSqure, -z * iNormSqure);
    }

    Vector3f QuatToEuler(const Quat& a) {
        Vector3d euler;

        double sinr_cosp = 2.f * (a.w * a.x + a.y * a.z);
        double cosr_cosp = 1.f - 2.f * (a.x * a.x + a.y * a.y);
        euler(0) = atan2(sinr_cosp, cosr_cosp);

        double sinp = 2.f * (a.w * a.y - a.z * a.x);
        euler(1) = fabs(sinp) >= 1.f ? copysign(acos(-1) / 2.f, sinp) : asin(sinp);

        double siny_cosp = 2.f * (a.w * a.z + a.x * a.y);
        double cosy_cosp = 1.f - 2.f * (a.y * a.y + a.z * a.z);
        euler(2) = atan2(siny_cosp, cosy_cosp);

        return euler;
    }

};

Quat findXQuat(const Quat& q1, const Quat& q2) { //q1 * x = q2
    Quat q1Inv = q1.inverse();
    Quat x = q1Inv * q2 * q1;
    return x;
}

k4a_float3_t QuatToEuler(k4a_quaternion_t quat) {
    k4a_float3_t euler;

    double sinr_cosp = 2.f * (quat.wxyz.w * quat.wxyz.x + quat.wxyz.y * quat.wxyz.z);
    double cosr_cosp = 1.f - 2.f * (quat.wxyz.x * quat.wxyz.x + quat.wxyz.y * quat.wxyz.y);
    euler.xyz.x = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.f * (quat.wxyz.w * quat.wxyz.y - quat.wxyz.z * quat.wxyz.x);
    euler.xyz.y = fabs(sinp) >= 1.f ? copysign(acos(-1) / 2.f, sinp) : asin(sinp);

    double siny_cosp = 2.f * (quat.wxyz.w * quat.wxyz.z + quat.wxyz.x * quat.wxyz.y);
    double cosy_cosp = 1.f - 2.f * (quat.wxyz.y * quat.wxyz.y + quat.wxyz.z * quat.wxyz.z);
    euler.xyz.z = atan2(siny_cosp, cosy_cosp);

    return euler;
}

Eigen::Vector3f quatToEuler(Quat quat) {
    Eigen::Vector3f result;
    float x, y, z;

    double sinr_cosp = 2.f * (quat.w * quat.x + quat.y * quat.z);
    double cosr_cosp = 1.f - 2.f * (quat.x * quat.x + quat.y * quat.y);
    x = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.f * (quat.w * quat.y - quat.z * quat.x);
    y = fabs(sinp) >= 1.f ? copysign(acos(-1) / 2.f, sinp) : asin(sinp);

    double siny_cosp = 2.f * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.f - 2.f * (quat.y * quat.y + quat.z * quat.z);
    z = atan2(siny_cosp, cosy_cosp);
    
    return result.setConstant(x, y, z);
}
class skeleton
{
public:
    Eigen::Vector3f pos;
    Eigen::Vector3f rot;

    Eigen::Vector3f transPos;
    Eigen::Vector3f transRot;

    Quat quat;

    int Count = 0;
};
int main()
{
    k4a_device_t device = NULL;
    k4a_device_t Subdevice = NULL;
    k4a_device_t tempdevice = NULL;

    bool IsInConnected = false;
    bool IsOutConnected = false;
    float LPFalpha = 0.9;
    float LPFVal = 0;

    skeleton master;
    skeleton sub;
    skeleton intergrated;

    kalmanFilter& KF = kalmanFilter::getInstance(1e-4, 10.f, 90.f, 90.f, 90.f);

    ifstream fin("calibration.txt");
    if (fin.is_open()) {
        fin >> sub.transPos[0] >> sub.transPos[1] >> sub.transPos[2];
        fin >> sub.transRot[0] >> sub.transRot[1] >> sub.transRot[2];
        fin.close();
    }

    printf("Installed Count ; %d", k4a_device_get_installed_count());


    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed");
    VERIFY(k4a_device_open(1, &Subdevice), "Open K4A Device failed - Sub");

    k4a_device_get_sync_jack(device, &IsInConnected, &IsOutConnected);
    if (IsInConnected)//device swap
    {
        tempdevice = device;
        device = Subdevice;
        Subdevice = tempdevice;
        printf("DEVICE SWAPED\n");
    }

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t SubConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    SubConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    SubConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    SubConfig.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
    VERIFY(k4a_device_start_cameras(Subdevice, &SubConfig), "Start K4A cameras failed! - Sub");

    printf("press any key to start master camera...\n");
    cin.get();
    
    k4a_device_configuration_t Config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    Config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    Config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    Config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
    VERIFY(k4a_device_start_cameras(device, &Config), "Start K4A cameras failed!");


    k4a_calibration_t Subcalibration;
    VERIFY(k4a_device_get_calibration(Subdevice, SubConfig.depth_mode, SubConfig.color_resolution, &Subcalibration), "Get depth camera calibration failed! - Sub");

    k4abt_tracker_t Subtracker = NULL;
    k4abt_tracker_configuration_t Subtracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    VERIFY(k4abt_tracker_create(&Subcalibration, Subtracker_config, &Subtracker), "Body tracker initialization failed! - Sub");
    k4abt_tracker_set_temporal_smoothing(Subtracker, 0.8);


    k4a_calibration_t calibration;
    VERIFY(k4a_device_get_calibration(device, Config.depth_mode, Config.color_resolution, &calibration),
        "Get depth camera calibration failed!");

    k4abt_tracker_t tracker = NULL;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    VERIFY(k4abt_tracker_create(&calibration, tracker_config, &tracker), "Body tracker initialization failed!");

    

    printf("READY\n");
    int frame_count = 0;
    do
    {
        frame_count++;
        system_clock::time_point start = system_clock::now();
        k4a_capture_t sensor_capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
            k4a_capture_release(sensor_capture); // Remember to release the sensor capture once you finish using it
            if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                // It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Add capture to tracker process queue timeout!\n");
                break;
            }
            else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
            {
                printf("Error! Add capture to tracker process queue failed!\n");
                break;
            }

            k4abt_frame_t body_frame = NULL;
            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
            if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
            {
                // Successfully popped the body tracking result. Start your processing

                size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
                //printf("%zu bodies are detected!\n", num_bodies);

                for (int i = 0; i < num_bodies; ++i)
                {
                    k4abt_body_t body;
                    k4abt_skeleton_t skeleton;
                    k4a_result_t bodyResult = k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
                    if (bodyResult == K4A_RESULT_SUCCEEDED)
                    {
                        k4abt_joint_t head = skeleton.joints[K4ABT_JOINT_HEAD];
                        k4a_float3_t position = head.position;
                        k4a_quaternion_t rotation = head.orientation;

                        //master 스켈레톤
                        master.pos = Eigen::Vector3f(position.xyz.x, position.xyz.y, position.xyz.z);
                        master.quat = Quat(rotation.wxyz.w, rotation.wxyz.x, rotation.wxyz.y, rotation.wxyz.z);
                        master.rot = quatToEuler(master.quat);

                        LPFVal = (LPFVal * LPFalpha) + (position.xyz.x * (1 - LPFalpha));
                        if (head.confidence_level == K4ABT_JOINT_CONFIDENCE_MEDIUM) {
                            intergrated.pos = master.pos;
                            intergrated.rot = master.rot;
                            KF.predict(1);
                            KF.update(intergrated.pos);
                            //KF.xHat을 읽어 필터링된 관절 데이터를 사용.
                        }
                        
                        cout << position.xyz.x << "\t\t" << LPFVal << "\t\t";
                        
                        
                    }
                }
                k4abt_frame_release(body_frame); // Remember to release the body frame once you finish using it
            }
            else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                //  It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Pop body frame result timeout!\n");
                break;
            }
            else
            {
                printf("Pop body frame result failed!\n");
                break;
            }
        }
        else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
        {
            // It should never hit time out when K4A_WAIT_INFINITE is set.
            printf("Error! Get depth frame time out!\n");
            break;
        }
        else
        {
            printf("Get depth capture returned error: %d\n", get_capture_result);
            break;
        }

        //Subdevice Tracking start
        get_capture_result = k4a_device_get_capture(Subdevice, &sensor_capture, K4A_WAIT_INFINITE);
        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            k4a_wait_result_t Subqueue_capture_result = k4abt_tracker_enqueue_capture(Subtracker, sensor_capture, K4A_WAIT_INFINITE);
            k4a_capture_release(sensor_capture);
            if (Subqueue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                printf("Error! Add capture to tracker process queue timeout! - Sub\n");
                break;
            }
            else if (Subqueue_capture_result == K4A_WAIT_RESULT_FAILED)
            {
                printf("Error! Add capture to tracker process queue failed! - Sub\n");
                break;
            }
            k4abt_frame_t body_frame = NULL;
            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(Subtracker, &body_frame, K4A_WAIT_INFINITE);
            if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
            {
                // Successfully popped the body tracking result. Start your processing

                size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
                //printf("%zu bodies are detected!\n", num_bodies);

                for (int i = 0; i < num_bodies; ++i)
                {
                    k4abt_body_t body;
                    k4abt_skeleton_t skeleton;
                    k4a_result_t bodyResult = k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
                    if (bodyResult == K4A_RESULT_SUCCEEDED)
                    {
                        k4abt_joint_t head = skeleton.joints[K4ABT_JOINT_HEAD];
                        k4a_float3_t position = head.position;
                        k4a_quaternion_t rotation = head.orientation;

                        //sub 스켈레톤
                        sub.pos = Eigen::Vector3f(position.xyz.x, position.xyz.y, position.xyz.z);
                        sub.quat = Quat(rotation.wxyz.w, rotation.wxyz.x, rotation.wxyz.y, rotation.wxyz.z);
                        sub.rot = quatToEuler(sub.quat);
                        if (head.confidence_level == K4ABT_JOINT_CONFIDENCE_MEDIUM)
                        {
                            intergrated.pos = sub.pos - sub.transPos;
                            intergrated.rot = sub.rot - sub.transRot;
                            KF.predict(1);
                            KF.update(intergrated.pos);
                            //KF.xHat을 읽어 필터링된 관절 데이터를 사용.
                        }
                       
                        //cout << all.py << endl;
                            
                        
                        //cout << "x : " << position.xyz.x << endl;
                        //통합할 때 차이값 측정
                        /*
                        sub.px = position.xyz.x - master.rx;
                        sub.py = position.xyz.y - master.ry;
                        sub.pz = position.xyz.z - master.rz;

                        cout << master.px - sub.px << ' ' << master.py - sub.py << ' ' << master.pz - sub.pz << endl;
                        */

                        
                        //통합 전 차이값 측정
                        /*
                        master.rx = position.xyz.x - master.px;
                        master.ry = position.xyz.y - master.py;
                        master.rz = position.xyz.z - master.pz;
                        cout << master.rx << ' ' << master.ry << ' ' << master.rz << endl;
                        */
                    }
                }

                k4abt_frame_release(body_frame); // Remember to release the body frame once you finish using it
            }
            else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                //  It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Pop body frame result timeout!\n");
                break;
            }
            else
            {
                printf("Pop body frame result failed!\n");
                break;
            }
        }
        else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
        {
            printf("Error! Get depth frame time out! - Sub\n");
            break;
        }
        else
        {
            printf("Get depth capture returned error : %d\n - Sub", get_capture_result);
            break;
        }

        duration<double> end = system_clock::now() - start;
        milliseconds milsec = duration_cast<seconds>(end);
        //cout << "milsec: " << (double)milsec.count() / 1000 << endl;
    } while (frame_count < 1000);

    printf("Finished body tracking processing!\n");

    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    k4abt_tracker_shutdown(Subtracker);
    k4abt_tracker_destroy(Subtracker);

    k4a_device_stop_cameras(Subdevice);
    k4a_device_close(Subdevice);
    k4a_device_stop_cameras(device);
    k4a_device_close(device);

    return 0;
}