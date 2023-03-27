// cpp_bodytracker.cpp : 이 파일에는 'main' 함수가 포함됩니다. 거기서 프로그램 실행이 시작되고 종료됩니다.
//
#pragma comment(lib, "k4a.lib")
#pragma commnet(lib, "k4abt.lib")
#include <iostream>
#include <k4a/k4a.h>
#include <k4abt.h>
using namespace std;

int main()
{
    //std::cout << "Helloㅐ World!\n";
    if (k4a_device_get_installed_count() == 0)
    {
        cout << "설치된 키넥트기기 없음\n";
        return 0;
    }
    k4a_device_t device = nullptr;
    int device_index = 0;
    k4a_device_open(device_index, &device);

    k4a_device_configuration_t camera_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    camera_config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    camera_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    camera_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    camera_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    //camera_config.wired_sync_mode = K4A_WIRED

    k4a_device_start_cameras(device, &camera_config);

    k4a_calibration_t calibration;
    k4a_device_get_calibration(device, camera_config.depth_mode, camera_config.color_resolution, &calibration);

    k4abt_tracker_t tracker = NULL;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    k4abt_tracker_create(&calibration, tracker_config, &tracker);
    int count = 0;
    //int available = 0;
    do
    {
        count++;
        k4a_capture_t capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &capture, K4A_WAIT_INFINITE);

        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            k4a_image_t colorImage = k4a_capture_get_color_image(capture);

            if (colorImage != nullptr)
            {
                unsigned char* colorData = k4a_image_get_buffer(colorImage);
                int colorWidth = k4a_image_get_width_pixels(colorImage);
                int colorHeight = k4a_image_get_height_pixels(colorImage);
                //컬러 이미지 해제 필요
            }
            k4a_image_t depthImage = k4a_capture_get_depth_image(capture);
            if (depthImage != nullptr)
            {
                unsigned short* depthData = reinterpret_cast<unsigned short*>(k4a_image_get_buffer(depthImage));
                int depthWidth = k4a_image_get_width_pixels(depthImage);
                int depthGeight = k4a_image_get_height_pixels(depthImage);
                //뎁스 이미지 해제 필요(release)
            }
            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, capture, K4A_WAIT_INFINITE);

            if (queue_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
            {
                k4abt_frame_t body_frame;
                k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);

                if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
                {
                    uint8_t bodyCount = static_cast<uint8_t>(k4abt_frame_get_num_bodies(body_frame));

                    for (int i = 0; i < bodyCount; ++i)
                    {
                        k4abt_skeleton_t skeleton;
                        k4a_result_t bodyResult = k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
                        if (bodyResult == K4A_RESULT_SUCCEEDED)
                        {
                            k4abt_joint_t head = skeleton.joints[K4ABT_JOINT_HAND_LEFT];
                            
                            
                            k4a_float3_t position = head.position;
                            k4abt_joint_confidence_level_t c_level = head.confidence_level;
                            cout << position.xyz.y << "\t";
                            if (head.confidence_level != K4ABT_JOINT_CONFIDENCE_MEDIUM)
                                cout << "LOW!\n";
                            else
                                cout << endl;
                            
                        }
                    }
                }
                k4abt_frame_release(body_frame);
            }
            k4a_image_release(depthImage);
            k4a_image_release(colorImage);
        }
        k4a_capture_release(capture);
    } while (count < 500);
    return 0;   
}