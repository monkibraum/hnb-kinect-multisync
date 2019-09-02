// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#define _CRT_SECURE_NO_WARNINGS
#define _SILENCE_CXX17_ITERATOR_BASE_CLASS_DEPRECATION_WARNING

#include <array>
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <Windows.h>

#include <k4a/k4a.h>
#include <Utilities.h>
#include <Window3dWrapper.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ole2.h>
#include <olectl.h>

#include <chrono>
#include <ctime> 

#include <cstdlib>
#include <bsoncxx/json.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/uri.hpp>
#include <cstdint>

#include <bsoncxx/builder/basic/array.hpp>
#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/builder/basic/kvp.hpp>
#include <bsoncxx/types.hpp>

#include <jsoncons/json.hpp>
#include "transformation_helpers.h"

using bsoncxx::builder::basic::kvp;
using bsoncxx::builder::basic::make_array;
using bsoncxx::builder::basic::make_document;

using namespace cv;


static bool color_to_depth_camera(k4a_transformation_t& transformation_handle,
	const k4a_image_t& depth_image,
	const k4a_image_t& color_image,
	k4a_image_t* image_handle

) {
	int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
	int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);
	k4a_image_t transformed_color_image = nullptr;
	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
		depth_image_width_pixels,
		depth_image_height_pixels,
		depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
		&transformed_color_image))
	{
		printf("Failed to create transformed color image\n");
		return false;
	}

	if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation_handle,
		depth_image,
		color_image,
		transformed_color_image))
	{
		printf("Failed to compute transformed color image\n");
		return false;
	}

	*image_handle = transformed_color_image;
	return true;
}

k4a_image_t depth_to_color_camera(k4a_transformation_t transformation_handle,
	const k4a_image_t depth_image,
	const k4a_image_t color_image

) {
	int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
	int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
	k4a_image_t transformed_depth_image = NULL;
	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
		color_image_width_pixels,
		color_image_height_pixels,
		color_image_width_pixels * (int)sizeof(uint16_t),
		&transformed_depth_image))
	{
		printf("Failed to create transformed depth image\n");
		return false;
	}

	if (K4A_RESULT_SUCCEEDED !=
		k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
	{
		printf("Failed to compute transformed depth image\n");
		return false;
	}

	return transformed_depth_image;
}

static bool point_cloud_color_to_depth(k4a_transformation_t transformation_handle,
	const k4a_image_t depth_image,
	const k4a_image_t color_image,
	std::string file_name)
{
	int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
	int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);
	k4a_image_t transformed_color_image = NULL;
	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
		depth_image_width_pixels,
		depth_image_height_pixels,
		depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
		&transformed_color_image))
	{
		printf("Failed to create transformed color image\n");
		return false;
	}

	k4a_image_t point_cloud_image = NULL;
	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
		depth_image_width_pixels,
		depth_image_height_pixels,
		depth_image_width_pixels * 3 * (int)sizeof(int16_t),
		&point_cloud_image))
	{
		printf("Failed to create point cloud image\n");
		return false;
	}

	if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation_handle,
		depth_image,
		color_image,
		transformed_color_image))
	{
		printf("Failed to compute transformed color image\n");
		return false;
	}

	if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
		depth_image,
		K4A_CALIBRATION_TYPE_DEPTH,
		point_cloud_image))
	{
		printf("Failed to compute point cloud\n");
		return false;
	}

	tranformation_helpers_write_point_cloud(point_cloud_image, transformed_color_image, file_name.c_str());

	k4a_image_release(transformed_color_image);
	k4a_image_release(point_cloud_image);

	return true;
}

static bool point_cloud_depth_to_color(k4a_transformation_t transformation_handle,
	const k4a_image_t depth_image,
	const k4a_image_t color_image,
	std::string file_name)
{
	// transform color image into depth camera geometry
	int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
	int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
	k4a_image_t transformed_depth_image = NULL;
	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
		color_image_width_pixels,
		color_image_height_pixels,
		color_image_width_pixels * (int)sizeof(uint16_t),
		&transformed_depth_image))
	{
		printf("Failed to create transformed depth image\n");
		return false;
	}

	k4a_image_t point_cloud_image = NULL;
	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
		color_image_width_pixels,
		color_image_height_pixels,
		color_image_width_pixels * 3 * (int)sizeof(int16_t),
		&point_cloud_image))
	{
		printf("Failed to create point cloud image\n");
		return false;
	}

	if (K4A_RESULT_SUCCEEDED !=
		k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
	{
		printf("Failed to compute transformed depth image\n");
		return false;
	}

	if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
		transformed_depth_image,
		K4A_CALIBRATION_TYPE_COLOR,
		point_cloud_image))
	{
		printf("Failed to compute point cloud\n");
		return false;
	}

	tranformation_helpers_write_point_cloud(point_cloud_image, color_image, file_name.c_str());

	k4a_image_release(transformed_depth_image);
	k4a_image_release(point_cloud_image);

	return true;
}

template<typename T>
inline void ConvertToGrayScaleImage(const T* imgDat, const int size, const int vmin, const int vmax, uint8_t* img)
{
	for (int i = 0; i < size; i++)
	{
		T v = imgDat[i];
		float colorValue = 0.0f;
		if (v <= vmin)
		{
			colorValue = 0.0f;
		}
		else if (v >= vmax)
		{
			colorValue = 1.0f;
		}
		else
		{
			colorValue = (float)(v - vmin) / (float)(vmax - vmin);
		}
		img[i] = (uint8_t)(colorValue * 255);
	}
}


void PrintAppUsage()
{
	printf("\n");
	printf(" Basic Navigation:\n\n");
	printf(" Rotate: Rotate the camera by moving the mouse while holding mouse left button\n");
	printf(" Pan: Translate the scene by holding Ctrl key and drag the scene with mouse left button\n");
	printf(" Zoom in/out: Move closer/farther away from the scene center by scrolling the mouse scroll wheel\n");
	printf(" Select Center: Center the scene based on a detected joint by right clicking the joint with mouse\n");
	printf("\n");
	printf(" Key Shortcuts\n\n");
	printf(" ESC: quit\n");
	printf(" h: help\n");
	printf(" b: body visualization mode\n");
	printf(" k: 3d window layout\n");
	printf("\n");
}

// Global State and Key Process Function
bool s_isRunning = true;
Visualization::Layout3d s_layoutMode = Visualization::Layout3d::OnlyMainView;
bool s_visualizeJointFrame = false;
bool writing_mode = false;

int64_t ProcessKey(void* /*context*/, int key)
{
	// https://www.glfw.org/docs/latest/group__keys.html
	switch (key)
	{
		// Quit
	case GLFW_KEY_ESCAPE:
		s_isRunning = false;
		break;
	case GLFW_KEY_K:
		s_layoutMode = (Visualization::Layout3d)(((int)s_layoutMode + 1) % (int)Visualization::Layout3d::Count);
		break;
	case GLFW_KEY_B:
		s_visualizeJointFrame = !s_visualizeJointFrame;
		break;
	case GLFW_KEY_H:
		PrintAppUsage();
		break;
	case GLFW_KEY_P:
		writing_mode = !writing_mode;
		if (writing_mode) {
			std::cout << "From now on, the program will write a captured snapshot to files ... /n Press P to toggle." << std::endl;
		}
		else {
			std::cout << "Writing mode disabled." << std::endl;
		}
		break;
	}
	return 1;
}

int64_t CloseCallback(void* /*context*/)
{
	s_isRunning = false;
	return 1;
}

k4a_depth_mode_t ParseDepthModeFromArg(int argc, char** argv)
{
	k4a_depth_mode_t depthCameraMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	if (argc > 1)
	{
		std::string inputArg(argv[1]);
		if (inputArg == std::string("NFOV_UNBINNED"))
		{
			depthCameraMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
		}
		else if (inputArg == std::string("WFOV_BINNED"))
		{
			depthCameraMode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
		}
		else
		{
			depthCameraMode = K4A_DEPTH_MODE_OFF;
		}
	}
	return depthCameraMode;
}

template<typename T> Mat create_mat_from_buffer(T* data, int width, int height, int channels = 1)
{
	Mat mat(height, width, CV_MAKETYPE(DataType<T>::type, channels));
	memcpy(mat.data, data, width * height * channels * sizeof(T));
	return mat;
}

int main(int argc, char** argv)
{

	//mongocxx::instance instance{}; // This should be done only once.
	//mongocxx::client client{ mongocxx::uri{"mongodb+srv://body_tracking_admin:hnbbody987@body-tracking-fzs8i.gcp.mongodb.net/test?retryWrites=true&w=majority"} };
	//mongocxx::database db = client["test"];
	//mongocxx::collection coll = db["body_tracking"];

	//std::cout << "DB 접속 완료" << std::endl;

	//int camera_num;
	//std::cout << "카메라 번호를 입력해주세요 숫자 ( 1 ~ 9 )" << std::endl;
	//std::cin >> camera_num;
	//std::cout << "camera_num : " << camera_num << std::endl;

	k4a_depth_mode_t depthCameraMode = ParseDepthModeFromArg(argc, argv);
	if (depthCameraMode == K4A_DEPTH_MODE_OFF)
	{
		return -1;
	}
	PrintAppUsage();

	k4a_device_t device0 = nullptr;
	//k4a_device_t device1 = nullptr;
	//k4a_device_t device2 = nullptr;

	//k4a_device_open(1, &device1);
	//k4a_device_open(2, &device2);
	k4a_device_open(0, &device0);


	// Start camera. Make sure depth camera is enabled.
	k4a_device_configuration_t deviceConfig0 = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	//deviceConfig0.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
	deviceConfig0.depth_mode = depthCameraMode;
	deviceConfig0.color_resolution = K4A_COLOR_RESOLUTION_720P;
	deviceConfig0.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; //K4A_IMAGE_FORMAT_COLOR_MJPG
	deviceConfig0.synchronized_images_only = true;
	deviceConfig0.camera_fps = K4A_FRAMES_PER_SECOND_5;

	//k4a_device_configuration_t deviceConfig1 = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	//deviceConfig1.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
	//deviceConfig1.depth_mode = depthCameraMode;
	//deviceConfig1.color_resolution = K4A_COLOR_RESOLUTION_720P;
	//deviceConfig1.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; //K4A_IMAGE_FORMAT_COLOR_MJPG
	//deviceConfig1.synchronized_images_only = true;
	//deviceConfig1.camera_fps = K4A_FRAMES_PER_SECOND_5;

	//k4a_device_configuration_t deviceConfig2 = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	//deviceConfig2.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
	//deviceConfig2.depth_mode = depthCameraMode;
	//deviceConfig2.color_resolution = K4A_COLOR_RESOLUTION_720P;
	//deviceConfig2.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; //K4A_IMAGE_FORMAT_COLOR_MJPG
	//deviceConfig2.synchronized_images_only = true;
	//deviceConfig2.camera_fps = K4A_FRAMES_PER_SECOND_5;

	/*k4a_device_start_cameras(device1, &deviceConfig1);
	k4a_device_start_cameras(device2, &deviceConfig2);*/
	k4a_device_start_cameras(device0, &deviceConfig0);



	// Get calibration information

	/*k4a_calibration_t sensorCalibration2;
	k4a_device_get_calibration(device2, deviceConfig2.depth_mode, deviceConfig2.color_resolution, &sensorCalibration2);

	k4a_calibration_t sensorCalibration1;
	k4a_device_get_calibration(device1, deviceConfig1.depth_mode, deviceConfig1.color_resolution, &sensorCalibration1);*/

	k4a_calibration_t sensorCalibration0;
	k4a_device_get_calibration(device0, deviceConfig0.depth_mode, deviceConfig0.color_resolution, &sensorCalibration0);


	/*int depthWidth2 = sensorCalibration2.depth_camera_calibration.resolution_width;
	int depthHeight2 = sensorCalibration2.depth_camera_calibration.resolution_height;

	int depthWidth1 = sensorCalibration1.depth_camera_calibration.resolution_width;
	int depthHeight1 = sensorCalibration1.depth_camera_calibration.resolution_height;*/

	int depthWidth0 = sensorCalibration0.depth_camera_calibration.resolution_width;
	int depthHeight0 = sensorCalibration0.depth_camera_calibration.resolution_height;

	// Initialize the 3d window controller
	Window3dWrapper window3d;
	window3d.Create("3D Visualization", sensorCalibration0);
	window3d.SetCloseCallback(CloseCallback);
	window3d.SetKeyCallback(ProcessKey);

	while (s_isRunning)
	{



		k4a_capture_t sensorCapture0 = nullptr;
		k4a_wait_result_t getCaptureResult0 = k4a_device_get_capture(device0, &sensorCapture0, 250); // timeout_in_ms is set to 0

		//k4a_capture_t sensorCapture1 = nullptr;
		//k4a_wait_result_t getCaptureResult1 = k4a_device_get_capture(device1, &sensorCapture1, 250); // timeout_in_ms is set to 0

		//k4a_capture_t sensorCapture2 = nullptr;
		//k4a_wait_result_t getCaptureResult2 = k4a_device_get_capture(device2, &sensorCapture2, 250); // timeout_in_ms is set to 0

		std::string dateTimeString{};
		std::string filename_depth_0{};
		std::string filename_color_0{};
		std::string filename_point_cloud{};
		/*std::string filename_depth_1{};
		std::string filename_color_1{};
		std::string filename_depth_2{};
		std::string filename_color_2{};
*/
		if (getCaptureResult0 == K4A_WAIT_RESULT_SUCCEEDED 
			//&& getCaptureResult1 == K4A_WAIT_RESULT_SUCCEEDED && getCaptureResult2 == K4A_WAIT_RESULT_SUCCEEDED
			)
		{

			k4a_transformation_t transformation0 = k4a_transformation_create(&sensorCalibration0);

			k4a_image_t colorImage0 = k4a_capture_get_color_image(sensorCapture0);
			k4a_image_t depthImage0 = k4a_capture_get_depth_image(sensorCapture0);
			k4a_image_t transformed_color_image0;
			color_to_depth_camera(transformation0, depthImage0, colorImage0, &transformed_color_image0);

			uint8_t* depthBuffer0 = k4a_image_get_buffer(depthImage0);
			//uint16_t* depthBuffer0 = reinterpret_cast<uint16_t*>(_depthBuffer0);
			uint8_t* colorBuffer0 = k4a_image_get_buffer(transformed_color_image0);


			uint64_t timestamp_usec = k4a_image_get_timestamp_usec(depthImage0);
			auto now = std::chrono::system_clock::now();
			std::time_t timestamp = std::chrono::system_clock::to_time_t(now);

			std::stringstream ssd0;
			ssd0 << "d0_" << timestamp << "_" << timestamp_usec << ".png";
			filename_depth_0 = ssd0.str();

			std::stringstream ssc0;
			ssc0 << "c0_" << timestamp << "_" << timestamp_usec << ".jpg";
			filename_color_0 = ssc0.str();

			std::stringstream ssp0;
			ssp0 << "pc_" << timestamp_usec << ".ply";
			filename_point_cloud = ssp0.str();
			//std::cout << filename_depth_0 << std::endl;

			//imwrite(filename_depth_0, create_mat_from_buffer<uint16_t>(depthBuffer0, depthWidth0, depthHeight0));
			const Mat depthImg0(depthHeight0, depthWidth0, CV_16UC1, depthBuffer0);
			imwrite(filename_depth_0, depthImg0);

			const Mat _colorImg0(depthHeight0, depthWidth0, CV_8UC4, colorBuffer0);
			Mat colorImg0; cvtColor(_colorImg0, colorImg0, COLOR_BGRA2BGR); imwrite(filename_color_0, colorImg0);

			point_cloud_depth_to_color(transformation0, depthImage0, colorImage0, filename_point_cloud);

			k4a_transformation_destroy(transformation0);


			////////////////////////////////////////////////////////////////////////////////////////////////


			//k4a_transformation_t transformation1 = k4a_transformation_create(&sensorCalibration1);

			//k4a_image_t colorImage1 = k4a_capture_get_color_image(sensorCapture1);
			//k4a_image_t depthImage1 = k4a_capture_get_depth_image(sensorCapture1);
			//k4a_image_t transformed_color_image1;
			//color_to_depth_camera(transformation1, depthImage1, colorImage1, &transformed_color_image1);

			//uint8_t* depthBuffer1 = k4a_image_get_buffer(depthImage1);
			////uint16_t* depthBuffer1 = reinterpret_cast<uint16_t*>(_depthBuffer1);
			//uint8_t* colorBuffer1 = k4a_image_get_buffer(transformed_color_image1);

			//std::stringstream ssd1;
			//ssd1 << "d1_" << timestamp << "_" << timestamp_usec << ".png";
			//filename_depth_1 = ssd1.str();

			//std::stringstream ssc1;
			//ssc1 << "c1_" << timestamp << "_" << timestamp_usec << ".jpg";
			//filename_color_1 = ssc1.str();

			//imwrite(filename_depth_1, create_mat_from_buffer<uint16_t>(depthBuffer1, depthWidth1, depthHeight1));

			/*const Mat depthImg1(depthHeight1, depthWidth1, CV_16UC1, depthBuffer1);
			imwrite(filename_depth_1, depthImg1);

			const Mat _colorImg1(depthHeight1, depthWidth1, CV_8UC4, colorBuffer1);
			Mat colorImg1; cvtColor(_colorImg1, colorImg1, COLOR_BGRA2BGR); imwrite(filename_color_1, colorImg1);

			k4a_transformation_destroy(transformation1);*/



			/////////////////////////////////////////////////////////////////////////////////////////


			//k4a_transformation_t transformation2 = k4a_transformation_create(&sensorCalibration2);

			//k4a_image_t colorImage2 = k4a_capture_get_color_image(sensorCapture2);
			//k4a_image_t depthImage2 = k4a_capture_get_depth_image(sensorCapture2);
			//k4a_image_t transformed_color_image2;
			//color_to_depth_camera(transformation2, depthImage2, colorImage2, &transformed_color_image2);

			//uint8_t* depthBuffer2 = k4a_image_get_buffer(depthImage2);
			////uint16_t* depthBuffer1 = reinterpret_cast<uint16_t*>(_depthBuffer1);
			//uint8_t* colorBuffer2 = k4a_image_get_buffer(transformed_color_image2);

			//std::stringstream ssd2;
			//ssd2 << "d2_" << timestamp << "_" << timestamp_usec << ".png";
			//filename_depth_2 = ssd2.str();

			//std::stringstream ssc2;
			//ssc2 << "c2_" << timestamp << "_" << timestamp_usec << ".jpg";
			//filename_color_2 = ssc2.str();

			////imwrite(filename_depth_1, create_mat_from_buffer<uint16_t>(depthBuffer1, depthWidth1, depthHeight1));

			//const Mat depthImg2(depthHeight2, depthWidth2, CV_16UC1, depthBuffer2);
			//imwrite(filename_depth_2, depthImg2);

			//const Mat _colorImg2(depthHeight2, depthWidth2, CV_8UC4, colorBuffer2);
			//Mat colorImg2; cvtColor(_colorImg2, colorImg2, COLOR_BGRA2BGR); imwrite(filename_color_2, colorImg2);

			//k4a_transformation_destroy(transformation2);


			////imwrite(filename0, create_mat_from_buffer<uint16_t>(depthBuffer0, depthWidth, depthHeight));
			////point_cloud_depth_to_color(transformation0, depthImage0, colorImage0, filename0);

			k4a_image_release(colorImage0);
			k4a_image_release(depthImage0);
			k4a_image_release(transformed_color_image0);

			/*k4a_image_release(colorImage1);
			k4a_image_release(depthImage1);
			k4a_image_release(transformed_color_image1);

			k4a_image_release(colorImage2);
			k4a_image_release(depthImage2);
			k4a_image_release(transformed_color_image2);*/

			k4a_capture_release(sensorCapture0);
			/*k4a_capture_release(sensorCapture1);
			k4a_capture_release(sensorCapture2);*/




			////k4a_transformation_t transformation1 = k4a_transformation_create(&sensorCalibration);

			////k4a_image_t colorImage1 = k4a_capture_get_color_image(sensorCapture1);
			////k4a_image_t depthImage1 = k4a_capture_get_depth_image(sensorCapture1);
			////
			////timestamp_usec = k4a_image_get_timestamp_usec(depthImage1);

			////std::stringstream ss1;
			////ss1 << "1_" << timestamp << "_" << timestamp_usec << ".ply";
			//////filename1 = ss1.str();

			//////point_cloud_depth_to_color(transformation1, depthImage1, colorImage1, filename1);

			////k4a_image_release(colorImage1);
			////k4a_image_release(depthImage1);
			////k4a_transformation_destroy(transformation1);

		}
		else
		{
			std::cout << "Get depth capture returned error: " << getCaptureResult0 << std::endl;
			//std::cout << "Get depth capture returned error: " << getCaptureResult1 << std::endl;
		}


	}

	std::cout << "Finished body tracking processing!!!!" << std::endl;

	window3d.Delete();

	k4a_device_stop_cameras(device0);
	k4a_device_close(device0);

	return 0;
}