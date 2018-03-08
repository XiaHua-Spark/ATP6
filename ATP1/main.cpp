#include <stdlib.h>
#include <iostream>
#include <fstream>   
#include <Windows.h>
#include <FlyCapture2.h>

#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> 

#include <string.h>
#include <tchar.h>
#include <math.h>

#include "NIDAQmx.h"

#pragma once


using namespace std;
using namespace FlyCapture2;

cv::Point2f GetSpotCenter(Camera& camera);

int setvoltage(TaskHandle taskhandle, float64 voltage);

vector<cv::Point2d> calibrate_controller(TaskHandle taskhandle, Camera& camera, cv::Point2f initMassCenter, int calib_start, int calib_stop, int calib_step, double factor);

int SetShotParameter(Camera& camera, int brightness_val, int gain_val, int shutter_val);


int sleeptime = 30;


int main()
{
	// difine global variable
	cv::Point2f currentMassCenter;
	cv::Point2f correctedMassCenter;

	std::vector<double> position_error_x(0);
	std::vector<double> position_error_y(0);
	std::vector<double> position_error_distance(0);

	// initialize camera
	Error error;
	Camera camera;
	CameraInfo camInfo;

	// initialize the adc
	TaskHandle	taskHandle_ao0 = 0;
	TaskHandle	taskHandle_ao1 = 0;

	DAQmxCreateTask("", &taskHandle_ao0);
	DAQmxCreateTask("", &taskHandle_ao1);

	DAQmxCreateAOVoltageChan(taskHandle_ao0, "Dev1/ao0", "", 0.0, 10.0, DAQmx_Val_Volts, "");
	DAQmxCreateAOVoltageChan(taskHandle_ao1, "Dev1/ao1", "", 0.0, 10.0, DAQmx_Val_Volts, "");

	DAQmxStartTask(taskHandle_ao0);
	DAQmxStartTask(taskHandle_ao1);

	
	// Connect the camera
	error = camera.Connect(0);
	error = camera.StartCapture();
	camera.GetCameraInfo(&camInfo);

	std::cout << camInfo.vendorName << " "
		      << camInfo.modelName  << " "
		      << camInfo.serialNumber << std::endl;

	SetShotParameter(camera, 0, 0, 1);

	// Get the image
	Image monoImage;
	Sleep(sleeptime);
	camera.RetrieveBuffer(&monoImage);


	// convert to OpenCV Mat
	unsigned int rowBytes = (double)monoImage.GetReceivedDataSize() / (double)monoImage.GetRows();
	cv::Mat image = cv::Mat(monoImage.GetRows(), monoImage.GetCols(), CV_8UC1, monoImage.GetData(), rowBytes);

	// display the image with modified parameters
	//cv::namedWindow("win");
	//cv::imshow("win", image);
	//cv::waitKey();
	



	// get init position for the spot
	cv::Point2f initMassCenter = GetSpotCenter(camera);

	cout << "initial position: " << initMassCenter << endl;




	int calib_start = 1;
	int calib_step = 1;
	int calib_stop = 100;

	// initialize the drift val table
	vector<cv::Point2d> drift_table_0(0), drift_table_1(0);
	drift_table_0 = calibrate_controller(taskHandle_ao0, camera, initMassCenter, calib_start, calib_stop, calib_step, 0.1);
	cout << "calibrate next controller: " << endl;
	drift_table_0 = calibrate_controller(taskHandle_ao1, camera, initMassCenter, calib_start, calib_stop, calib_step, 0.1);

	DAQmxStopTask(taskHandle_ao0);
	DAQmxStopTask(taskHandle_ao1);

	// save calibration data to file
	std::vector<double> offset_x(0);
	std::vector<double> offset_y(0);
	ofstream outputFile;
	outputFile.open("outputFile_controller0.txt", std::ios::out);
	for (size_t ii = 0; ii < drift_table_0.size(); ++ii)
	{
		outputFile << drift_table_0[ii].x << "," << drift_table_0[ii].y << std::endl;
		offset_x.push_back(drift_table_0[ii].x);
		offset_y.push_back(drift_table_0[ii].y);
	}
	outputFile.close();
	
	// find the correspondence between the port and offset 
	cv::Mat x_mean_mat, y_mean_mat, x_std_mat, y_std_mat;
	double x_std, y_std, x_mean, y_mean;
	cv::meanStdDev(offset_x, x_mean_mat, x_std_mat);
	cv::meanStdDev(offset_y, y_mean_mat, y_std_mat);
	x_std = x_std_mat.at<double>(0, 0);
	y_std = y_std_mat.at<double>(0, 0);
	//x_mean = x_mean_mat.at<double>(0, 0);
	//y_mean = y_mean_mat.at<double>(0, 0);
	x_mean = offset_x[offset_x.size() - 1] - offset_x[5];
	y_mean = offset_y[offset_y.size() - 1] - offset_y[5];

	short sign_x = 1, sign_y = 1;

	// if the controller[0] controls y axis
	if (x_std < y_std)
	{
		DAQmxCreateAOVoltageChan(taskHandle_ao0, "Dev1/ao1", "", 0.0, 10.0, DAQmx_Val_Volts, "");
		DAQmxCreateAOVoltageChan(taskHandle_ao1, "Dev1/ao0", "", 0.0, 10.0, DAQmx_Val_Volts, "");

		if (y_mean < 0)
		{
			sign_y = -1;
		}
	}
	else
	{
		if (x_mean < 0)
		{
			sign_x = -1;
		}
	}

	// save calibration data to file
	outputFile.open("outputFile_controller1.txt", std::ios::out);
	offset_x.clear();
	offset_y.clear();
	for (size_t ii = 0; ii < drift_table_1.size(); ++ii)
	{
		outputFile << drift_table_1[ii].x << "," << drift_table_1[ii].y << std::endl;
		offset_x.push_back(drift_table_1[ii].x);
		offset_y.push_back(drift_table_1[ii].y);
	}
	outputFile.close();
	cv::meanStdDev(offset_x, x_mean_mat, x_std_mat);
	cv::meanStdDev(offset_y, y_mean_mat, y_std_mat);
	//x_mean = x_mean_mat.at<double>(0, 0);
	//y_mean = y_mean_mat.at<double>(0, 0);
	x_mean = offset_x[offset_x.size() - 1] - offset_x[5];
	y_mean = offset_y[offset_y.size() - 1] - offset_y[5];

	if (x_std < y_std)
	{
		if (x_mean < 0)
		{
			sign_x = -1;
		}
	}
	else
	{
		if (y_mean < 0)
		{
			sign_y = -1;
		}
	}

	DAQmxStartTask(taskHandle_ao0);
	DAQmxStartTask(taskHandle_ao1);

	// capture loop
	char key = 0;
	float64 error_tolerance = 0.5;
	float64 kp_0 = 5, kp_1 = 5;    // Proportion
	float64 ki_0 = 5.0, ki_1 = 5.0; // Integral    
	float64 kd_0 = 0.0, kd_1 = 0.0; // Derivative
	

	//std::vector<cv::Point> historyMassCenter(0);

	while (key != 'q')
	{
		currentMassCenter = GetSpotCenter(camera);
		//log each movement of mass center to show the trajactory 
		
		// x,y location of the mass center
		//double xloc = currentMassCenter.x;
		//double yloc = currentMassCenter.y;
		float64 v0 = 5.0, v1 = 5.0;
		float64 x_error_present = currentMassCenter.x - initMassCenter.x;
		float64 y_error_present = currentMassCenter.y - initMassCenter.y;
		float64 position_error_present = sqrt(pow(x_error_present, 2) + pow(y_error_present, 2));
		float64 delta_v0 = 0, delta_v1 = 0;
	
		float64 x_error_last = 0, y_error_last = 0;
		float64 x_error_previous = 0, y_error_previous = 0;


		int run_times = 0;
		while ( position_error_present > error_tolerance && run_times < 300 )
		{


			delta_v0 = kp_0 * (x_error_present - x_error_last)
				+ ki_0 * x_error_present
				+ kd_0 * (x_error_present - 2 * x_error_last + x_error_previous);
			delta_v1 = kp_1 * (y_error_present - y_error_last)
				+ ki_1 * y_error_present
				+ kd_1 * (y_error_present - 2 * y_error_last + y_error_previous);
	        v0 -= sign_x * delta_v0;
			v1 -= sign_y * delta_v1;
		
			run_times++;

			// set output voltage
			setvoltage(taskHandle_ao0, v0);
			setvoltage(taskHandle_ao1, v1);
			Sleep(sleeptime);

			// read corrected position
		    correctedMassCenter = GetSpotCenter(camera);
			//historyMassCenter.push_back(correctedMassCenter);

			// x location of the corrected mass center
			//xloc = correctedMassCenter.x;		
			x_error_previous = x_error_last;
			x_error_last = x_error_present;
			x_error_present = correctedMassCenter.x - initMassCenter.x;
			//yloc = correctedMassCenter.y;
			y_error_previous = y_error_last;
			y_error_last = y_error_present;
			y_error_present = correctedMassCenter.y - initMassCenter.y;
			position_error_present = sqrt(pow(x_error_present,2) + pow(y_error_present, 2));

			position_error_x.push_back(x_error_present);
			position_error_y.push_back(y_error_present);
			position_error_distance.push_back(position_error_present);
			cout << "iteration counts: " << position_error_distance.size() << endl;
		}

		//for debug 
		fstream outputFile;
		outputFile.open("output_error.txt", std::ios::out);
		for (short ii = 0; ii < position_error_distance.size(); ii++)
		{
			outputFile << position_error_x[ii] << "," << position_error_y[ii] << "," << position_error_distance[ii] << std::endl;
		}
		outputFile.close();

		// clear x,y,distance error log
		position_error_x.clear();
		position_error_y.clear();
		position_error_distance.clear();

		//for (short ii = 0; ii < historyMassCenter.size(); ii++)
		//{
		//	cv::circle(image, historyMassCenter[ii], 1, (128, 128, 0), -1);
		//	
		//	/*
		//	cv::Point Orig;
		//	Orig.x = historyMassCenter[ii].x - 10;
		//	Orig.y = historyMassCenter[ii].y - 10;
		//	
		//	cv::putText(image, "center", Orig, cv::FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1);
		//	*/
		//
		//}
		//
		//cv::namedWindow("trajactory");
		//cv::imshow("trajactory", image);
		//cv::waitKey();

		//cv::imwrite("trajactory_Image.jpg", image);
		
	}


	error = camera.StopCapture();
	if (error != PGRERROR_OK)
	{
		// This may fail when the camera was removed, so don't show 
		// an error message
	}

	DAQmxStopTask(taskHandle_ao0);
	DAQmxStopTask(taskHandle_ao1);

	return 0;
}




cv::Point2f GetSpotCenter(Camera& camera)
{

	Image monoImage;
	camera.RetrieveBuffer(&monoImage);
	//Sleep(100);

	// convert to OpenCV Mat
	unsigned int rowBytes = (double)monoImage.GetReceivedDataSize() / (double)monoImage.GetRows();
	cv::Mat image = cv::Mat(monoImage.GetRows(), monoImage.GetCols(), CV_8UC1, monoImage.GetData(), rowBytes);


	// blur it slightly
	//cv::Mat blurred;
	//cv::GaussianBlur(image, blurred, cv::Size(3, 3), 0);

	// threshold it
	cv::Mat thresh;
	cv::threshold(image, thresh, 200, 255, cv::THRESH_BINARY);

	// find contours in the thresholded image
	cv::Mat dst = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);

	vector< vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;

	cv::findContours(thresh, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	// find the largest contour and its index
	int largest_area = 0;
	int largest_contour_index = 0;

	for (size_t i = 0; i < contours.size(); i++)  // iterate through each contour.
	{
		double area = contourArea(contours[i]);  //  Find the area of contour

		if (area > largest_area)
		{
			largest_area = area;
			largest_contour_index = i;           //Store the index of largest contour
		}
	}

	int lci = largest_contour_index;

	// Get the moments
	cv::Moments mu;
	mu = moments(contours[lci], false);

	// Get the mass centers:
	cv::Point2f mc;
	mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

	// draw contour
	/*cv::drawContours(image, contours, -1, (255, 255, 0), 2);
	cv::namedWindow("contour");
	cv::imshow("contour", image);
	cv::waitKey();*/

	return mc;
}

int setvoltage(TaskHandle taskhandle, float64 voltage)
{
	
	if (voltage >= 10)
	{
		voltage = 10;
		cout << "voltage too high" << endl;
	}

	if (voltage <= 0)
	{
		voltage = 0;
		cout << "voltage too low" << endl;
	}
	DAQmxWriteAnalogScalarF64(taskhandle, 0, 0, voltage, NULL);
	return 0;
}

vector<cv::Point2d> calibrate_controller(TaskHandle taskhandle, Camera& camera, cv::Point2f initMassCenter, int calib_start, int calib_stop, int calib_step, double factor)
{
	vector<cv::Point2d> calib_table(0);
	// calibration loop for pizo_serialno
	for (int j = calib_start; j <= calib_stop; j += calib_step) // calibrate with calib_iter 
	{

		// change voltage
		float64 data = j * factor;
		DAQmxWriteAnalogScalarF64(taskhandle, 0, 0, data, NULL);


		Sleep(sleeptime);

		cv::Point2f tempMassCenter = GetSpotCenter(camera);

		cout << "temp position: " << tempMassCenter << endl;

		// save current drift to drift table
		calib_table.push_back(tempMassCenter - initMassCenter);

	}
	return calib_table;

}

int SetShotParameter(Camera& camera, int brightness_val, int gain_val, int shutter_val)
{
	//set bright to minima val
	Property brightness;
	brightness.type = BRIGHTNESS;
	brightness.absControl = false;
	brightness.valueA = brightness_val;
	camera.SetProperty(&brightness);

	// set gain to minima val
	Property gain;
	gain.type = GAIN;
	gain.absControl = false;
	gain.valueA = gain_val;
	camera.SetProperty(&gain);

	// adaptive shutter time ensure the max val > 200
	Image monoImage;
	double min = 0.0;
	double max = 0.0;
	
	while (max < 200) 
	{
		camera.RetrieveBuffer(&monoImage);

		// convert to OpenCV Mat
		unsigned int rowBytes = (double)monoImage.GetReceivedDataSize() / (double)monoImage.GetRows();
		cv::Mat image = cv::Mat(monoImage.GetRows(), monoImage.GetCols(), CV_8UC1, monoImage.GetData(), rowBytes);
		
		// get the max val of the image
		cv::minMaxLoc(image, &min, &max);
		
		shutter_val ++;
		cout << "using shutter value: " << shutter_val << endl;
	}

	// change shutter using camera internal unit
	Property shutter;
	shutter.type = SHUTTER;
	shutter.absControl = false;
	shutter.valueA = shutter_val;
	camera.SetProperty(&shutter);

	cout << "shutter value: " << shutter_val << endl;
	return 0;
}

