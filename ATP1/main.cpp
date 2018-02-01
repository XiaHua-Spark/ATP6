#include <stdlib.h>
#include <iostream>
#include <fstream>   

#include <FlyCapture2.h>

#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> 

#include <string.h>
#include <tchar.h>
#include <math.h>
#include "Thorlabs.MotionControl.KCube.Piezo.h"
#pragma once

using namespace std;
using namespace FlyCapture2;

cv::Point2f GetSpotCenter(Camera& camera);

int setvoltage(char* piezo_serialno, double voltage, double max_voltage);

vector<cv::Point2d> calibrate_controller(char* piezo_serialno, Camera& camera, cv::Point2f initMassCenter, int calib_start, int calib_stop, int calib_step, double maximumoutput);

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

	// initialize the kpz101
	// Build list of the connected device
	short piezo_connect_check = TLI_BuildDeviceList();
	if (piezo_connect_check != 0 )
	{
		cout << "No piezo controller detected!" << endl;
		exit(0);
	}
	//get device list size
	short n = TLI_GetDeviceListSize();
	printf("Found Device matched: %d!\r\n", n);
	char *context = NULL;
	//get BBD serial numbers
	char serialNos[100] = { '0' };
	TLI_GetDeviceListByTypeExt(serialNos, 100, 29);
	std::vector<std::string> serialNo_total(0);

	//output list of matching devices
	char *p = strtok_s(serialNos, ",", &context);
	while (p != NULL)
	{
		TLI_DeviceInfo deviceInfo;
		//get device info form device
		TLI_GetDeviceInfo(p, &deviceInfo);
		//get strings from device info structure
		char desc[65];
		strncpy_s(desc, deviceInfo.description, 64);
		desc[64] = '\0';
		char serialNo[9];
		strncpy_s(serialNo, deviceInfo.serialNo, 8);
		serialNo[8] = '\0';
		std::string serialNo_temp;
		serialNo_temp = std::string(serialNo);
		serialNo_total.push_back(serialNo_temp);
		// output
		printf("Found Device %s=%s : %s\r\n", p, serialNo, desc);
		p = strtok_s(NULL, ",", &context);
	}
	char testSerialNo[2][9] = { '\0' };
	double maximumoutput = 150;
	short setmaximumoutput = maximumoutput * 10;
	for (short i = 0; i < n; i++)
	{
		strncpy_s(testSerialNo[i], serialNo_total[i].c_str(), serialNo_total[i].length());
		PCC_Open(testSerialNo[i]);
		PCC_CheckConnection(testSerialNo[i]);
		PCC_Identify(testSerialNo[i]);
		PCC_Enable(testSerialNo[i]);
		PCC_SetZero(testSerialNo[i]);
		PCC_SetPositionControlMode(testSerialNo[i], PZ_ControlModeTypes::PZ_OpenLoop);
		PCC_SetVoltageSource(testSerialNo[i], PZ_InputSourceFlags::PZ_Potentiometer);
		PCC_SetMaxOutputVoltage(testSerialNo[i], setmaximumoutput);
	}
	
	// set initial voltage
	setvoltage(testSerialNo[0], 75.0, maximumoutput);
	setvoltage(testSerialNo[1], 75.0, maximumoutput);

	Sleep(sleeptime);

	// get init position for the spot
	cv::Point2f initMassCenter = GetSpotCenter(camera);

	cout << "initial position: " << initMassCenter << endl;




	int calib_start = 1;
	int calib_step = 1;
	int calib_stop = 150;

	// initialize the drift val table
	vector<cv::Point2d> drift_table_0(0), drift_table_1(0);
	drift_table_0 = calibrate_controller(testSerialNo[0], camera, initMassCenter, calib_start, calib_stop, calib_step, maximumoutput);
	cout << "calibrate next controller: " << endl;
	drift_table_1 = calibrate_controller(testSerialNo[1], camera, initMassCenter, calib_start, calib_stop, calib_step, maximumoutput);
		
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
		char testSerialNo_temp[9] = { '\0' };
		strcpy_s(testSerialNo_temp, testSerialNo[0]);
		strcpy_s(testSerialNo[0], testSerialNo[1]);
		strcpy_s(testSerialNo[1], testSerialNo_temp);
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


	// start ATP from a bad position

	//setvoltage(testSerialNo[0], 75.0, maximumoutput);
	//setvoltage(testSerialNo[1], 75.0, maximumoutput);

	//Sleep(sleeptime);

	// capture loop
	char key = 0;
	double error_tolerance = 0.5;
	double kp_0 = 5, kp_1 = 5;    // Proportion
	double ki_0 = 5.0, ki_1 = 5.0; // Integral    
	double kd_0 = 0.0, kd_1 = 0.0; // Derivative
	

	//std::vector<cv::Point> historyMassCenter(0);

	while (key != 'q')
	{
		currentMassCenter = GetSpotCenter(camera);
		//log each movement of mass center to show the trajactory 
		
		// x,y location of the mass center
		//double xloc = currentMassCenter.x;
		//double yloc = currentMassCenter.y;
		double v0 = 75.0, v1 = 75.0;
		double x_error_present = currentMassCenter.x - initMassCenter.x;
		double y_error_present = currentMassCenter.y - initMassCenter.y;
		double position_error_present = sqrt(pow(x_error_present, 2) + pow(y_error_present, 2));
		double delta_v0 = 0, delta_v1 = 0;
	
		double x_error_last = 0, y_error_last = 0;
		double x_error_previous = 0, y_error_previous = 0;


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
			setvoltage(testSerialNo[0], v0, maximumoutput);
			setvoltage(testSerialNo[1], v1, maximumoutput);
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

	for (int m = 0; m < n; m++)
	{
		PCC_Close(testSerialNo[m]);
	}


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
	cv::threshold(image, thresh, 20, 255, cv::THRESH_BINARY);

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

	return mc;
}

int setvoltage(char* piezo_serialno, double voltage, double max_voltage)
{
	
	if (voltage >= 150)
	{
		voltage = 150;
		cout << "voltage too high" << endl;
	}

	if (voltage <= 0)
	{
		voltage = 0;
		cout << "voltage too low" << endl;
	}
	short setoutputvoltage = short(voltage / max_voltage * 32767.0);
	PCC_SetOutputVoltage(piezo_serialno, setoutputvoltage);
	return 0;
}

vector<cv::Point2d> calibrate_controller(char* piezo_serialno, Camera& camera, cv::Point2f initMassCenter, int calib_start, int calib_stop, int calib_step, double maximumoutput)
{
	vector<cv::Point2d> calib_table(0);
	// calibration loop for pizo_serialno
	for (int j = calib_start; j <= calib_stop; j += calib_step) // calibrate with calib_iter 
	{

		// change voltage
		setvoltage(piezo_serialno, double(j), maximumoutput);
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
	//set bright
	Property brightness;
	brightness.type = BRIGHTNESS;
	brightness.absControl = false;
	brightness.valueA = brightness_val;
	camera.SetProperty(&brightness);

	// set gain to minma val
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

