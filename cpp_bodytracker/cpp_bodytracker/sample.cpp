#pragma comment(lib, "k4a.lib")
#pragma commnet(lib, "k4abt.lib")
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <k4a/k4a.h>
#include <k4abt.h>
#include <chrono>
using namespace std;
using namespace chrono;

#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }                                                                                                    \

int main()
{
    k4a_device_t device = NULL;

    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed - SUB");




   

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t Config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    Config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    Config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    Config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
    VERIFY(k4a_device_start_cameras(device, &Config), "Start K4A cameras failed!");


    k4a_calibration_t calibration;
    VERIFY(k4a_device_get_calibration(device, Config.depth_mode, Config.color_resolution, &calibration),
        "Get depth camera calibration failed!");

    k4abt_tracker_t tracker = NULL;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    VERIFY(k4abt_tracker_create(&calibration, tracker_config, &tracker), "Body tracker initialization failed!");



    int frame_count = 0;
    do
    {
        system_clock::time_point start = system_clock::now();
        k4a_capture_t sensor_capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            frame_count++;
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
                        k4a_quaternion_t ori = head.orientation;
                        k4abt_joint_confidence_level_t c_level = head.confidence_level;
                        k4abt_joint_confidence_level_t a = K4ABT_JOINT_CONFIDENCE_LEVELS_COUNT;
                        //out << "x : " << position.xyz.x << endl;
                        //cout << ori.wxyz.x << "\t" << ori.wxyz.y << "\t" << ori.wxyz.z << "\t" << ori.wxyz.w << endl;
                        cout << c_level << endl;
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
        duration<double> end = system_clock::now() - start;
        milliseconds milsec = duration_cast<seconds>(end);
        //cout << "milsec: " << (double)milsec.count() / 1000 << endl;
    } while (frame_count < 1000);

    printf("Finished body tracking processing!\n");

    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    k4a_device_stop_cameras(device);
    k4a_device_close(device);

    return 0;
}