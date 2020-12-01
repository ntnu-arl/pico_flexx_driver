/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * This file is part of pico_flexx_driver.
 *
 * pico_flexx_driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * pico_flexx_driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with pico_flexx_driver.  If not, see <http://www.gnu.org/licenses/>.
 */

//Customization
//#define PUB_IMAGES
//#define PUB_CLOUD

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include <ros/ros.h>
#include <ros/console.h>
#include <nodelet/nodelet.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <royale.hpp>

#include <dynamic_reconfigure/server.h>
#include <pico_flexx_driver/pico_flexx_driverConfig.h>

// #include <Eigen/Dense>
#include <eigen3/Eigen/Dense>

#define PF_DEFAULT_NS       "pico_flexx"
#define PF_TF_LINK          "_link"
#define PF_TF_OPT_FRAME     "_optical_frame"
#define PF_TOPIC_INFO       "/camera_info"
#define PF_TOPIC_MONO8      "/image_mono8"
#define PF_TOPIC_MONO16     "/image_mono16"
#define PF_TOPIC_DEPTH      "/image_depth"
#define PF_TOPIC_NOISE      "/image_noise"
#define PF_TOPIC_CLOUD      "/points"

// fix for royale sdk definitions
#undef __PRETTY_FUNCTION__

// Set this to '0' to disable the extended colored output
#define EXTENDED_OUTPUT 1

#if EXTENDED_OUTPUT

#define NO_COLOR        "\033[0m"
#define FG_BLACK        "\033[30m"
#define FG_RED          "\033[31m"
#define FG_GREEN        "\033[32m"
#define FG_YELLOW       "\033[33m"
#define FG_BLUE         "\033[34m"
#define FG_MAGENTA      "\033[35m"
#define FG_CYAN         "\033[36m"

#define OUT_FUNCTION(NAME) ([](const std::string &name)\
{ \
	size_t end = name.rfind('(');\
	if(end == std::string::npos) end = name.size();\
	size_t begin = 1 + name.rfind(' ', end);\
	return name.substr(begin, end - begin);\
}(NAME))
#define OUT_AUX(FUNC_COLOR, MSG_COLOR, STREAM, MSG) STREAM(FUNC_COLOR "[" << OUT_FUNCTION(__PRETTY_FUNCTION__) << "] " MSG_COLOR << MSG << NO_COLOR)

#define OUT_DEBUG(msg) OUT_AUX(FG_BLUE, NO_COLOR, ROS_DEBUG_STREAM, msg)
#define OUT_INFO(msg) OUT_AUX(FG_GREEN, NO_COLOR, ROS_INFO_STREAM, msg)
#define OUT_WARN(msg) OUT_AUX(FG_YELLOW, FG_YELLOW, ROS_WARN_STREAM, msg)
#define OUT_ERROR(msg) OUT_AUX(FG_RED, FG_RED, ROS_ERROR_STREAM, msg)

#else

#define NO_COLOR        ""
#define FG_BLACK        ""
#define FG_RED          ""
#define FG_GREEN        ""
#define FG_YELLOW       ""
#define FG_BLUE         ""
#define FG_MAGENTA      ""
#define FG_CYAN         ""

#define OUT_DEBUG(msg) ROS_DEBUG_STREAM(msg)
#define OUT_INFO(msg) ROS_INFO_STREAM(msg)
#define OUT_WARN(msg) ROS_WARN_STREAM(msg)
#define OUT_ERROR(msg) ROS_WARN_STREAM(msg)

#endif

class PicoFlexx : public royale::IDepthDataListener, public royale::IExposureListener2
{
private:
	enum Topics
	{
		CAMERA_INFO = 0,
		MONO_8,
		MONO_16,
		DEPTH,
		NOISE,
		CLOUD,
		COUNT
	};

	ros::NodeHandle nh, priv_nh;
	sensor_msgs::CameraInfo cameraInfo;
	std::vector<std::vector<ros::Publisher>> publisher;

	ros::Publisher points_publisher;
	ros::Publisher free_points_publisher;

	std::vector<std::vector<bool>> status;

	bool pub_status;

	std::vector<float> frustum_endpoints_x;
	std::vector<float> frustum_endpoints_y;
	std::vector<float> frustum_endpoints_z;

	boost::recursive_mutex lockServer;
	dynamic_reconfigure::Server<pico_flexx_driver::pico_flexx_driverConfig> server;
	pico_flexx_driver::pico_flexx_driverConfig configMin, configMax, config;
	std::vector<int> cbExposureTime;

	std::unique_ptr<royale::ICameraDevice> cameraDevice;
	std::unique_ptr<royale::DepthData> data;

	std::mutex lockStatus, lockData, lockTiming;
	std::condition_variable cvNewData;
	bool running, newData;
	std::vector<bool> ignoreNewExposure;
	uint64_t frame, framesPerTiming, processTime, delayReceived;
	std::string baseNameTF;
	std::chrono::high_resolution_clock::time_point startTime;
	std::thread threadProcess;

public:
	PicoFlexx(const ros::NodeHandle &nh = ros::NodeHandle(), const ros::NodeHandle &priv_nh = ros::NodeHandle("~"))
		: royale::IDepthDataListener(), royale::IExposureListener2(), nh(nh), priv_nh(priv_nh), server(lockServer)
	{
		cbExposureTime.resize(2);
		running = false;
		newData = false;
		ignoreNewExposure.resize(2);
		frame = 0;
		framesPerTiming = 25;
		processTime = 0;
		delayReceived = 0;
		publisher.resize(2);
		publisher[0].resize(COUNT);
		publisher[1].resize(COUNT);
		status.resize(2);
		status[0].resize(COUNT, false);
		status[1].resize(COUNT, false);

		config.use_case = 0;
		config.exposure_mode = 0;
		config.exposure_time = 1000;
		config.exposure_mode_stream2 = 0;
		config.exposure_time_stream2 = 1000;
		config.max_noise = 0.07;
		config.range_factor = 2.0;

		configMin.use_case = 0;
		configMin.exposure_mode = 0;
		configMin.exposure_time = 50;
		configMin.exposure_mode_stream2 = 0;
		configMin.exposure_time_stream2 = 50;
		configMin.max_noise = 0.0;
		configMin.range_factor = 0.0;

		config.min_depth = 0.0;

		configMax.use_case = 5;
		configMax.exposure_mode = 1;
		configMax.exposure_time = 2000;
		configMax.exposure_mode_stream2 = 1;
		configMax.exposure_time_stream2 = 2000;
		configMax.max_noise = 0.10;
		configMax.range_factor = 7.0;

		configMax.min_depth = 10.0;

	}

	~PicoFlexx()
	{
	}

	void start()
	{
		if(!initialize())
		{
			return;
		}
		running = true;

		threadProcess = std::thread(&PicoFlexx::process, this);

		OUT_INFO("waiting for clients to connect");
	}

	void stop()
	{
		cameraDevice->stopCapture();
		running = false;

		threadProcess.join();
		return;
	}

	void onNewData(const royale::DepthData *data)
	{
		std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();

		lockTiming.lock();
		delayReceived += (now.time_since_epoch() - std::chrono::duration_cast<std::chrono::nanoseconds>(data->timeStamp)).count();
		lockTiming.unlock();

		lockData.lock();
		this->data = std::unique_ptr<royale::DepthData>(new royale::DepthData);
		this->data->version = data->version;
		this->data->timeStamp = data->timeStamp;
		this->data->streamId = data->streamId;
		this->data->width = data->width;
		this->data->height = data->height;
		this->data->exposureTimes = data->exposureTimes;
		this->data->points = data->points;
		newData = true;
		lockData.unlock();
		cvNewData.notify_one();
	}

	void onNewExposure(const uint32_t newExposureTime, const royale::StreamId streamId)
	{
		size_t streamIndex;
		if (!findStreamIndex(streamId, streamIndex))
		{
			return;
		}

		if(ignoreNewExposure[streamIndex])
		{
			ignoreNewExposure[streamIndex] = false;
			cbExposureTime[streamIndex] = (int)newExposureTime;
			return;
		}

		if (streamIndex == 0)
		{
			if(config.exposure_time == (int)newExposureTime)
			{
				return;
			}

			OUT_DEBUG("exposure changed: " FG_YELLOW << newExposureTime);
			config.exposure_time = (int)newExposureTime;
		}
		else if (streamIndex == 1)
		{
			if(config.exposure_time_stream2 == (int)newExposureTime)
			{
				return;
			}

			OUT_DEBUG("exposure changed (stream2): " FG_YELLOW << newExposureTime);
			config.exposure_time_stream2 = (int)newExposureTime;
		}
		server.updateConfig(config);
	}

	void callbackTopicStatus()
	{
		lockStatus.lock();
		bool clientsConnected = false;
		for(size_t i = 0; i < 2; ++i)
		{
			for(size_t j = 0; j < COUNT; ++j)
			{
				status[i][j] = publisher[i][j].getNumSubscribers() > 0;
				clientsConnected = clientsConnected || status[i][j];
			}
		}

		//Customization
		pub_status = points_publisher.getNumSubscribers() > 0 || free_points_publisher.getNumSubscribers() > 0;
		clientsConnected |= pub_status;
		//Customization

		bool isCapturing(false);
		cameraDevice->isCapturing(isCapturing);
		if(clientsConnected && !isCapturing)
		{
			OUT_INFO("client connected. starting device...");

			lockTiming.lock();
			processTime = 0;
			frame = 0;
			delayReceived = 0;
			lockTiming.unlock();
			ignoreNewExposure[0] = config.exposure_mode == 0; // ignore if manual mode
			ignoreNewExposure[1] = config.exposure_mode_stream2 == 0;

			if(cameraDevice->startCapture() != royale::CameraStatus::SUCCESS)
			{
				OUT_ERROR("could not start capture!");
				running = false;
				ros::shutdown();
			}

			royale::Vector<royale::StreamId> streams;
			cameraDevice->getStreams(streams);

			if(config.exposure_mode == 0
				 && streams.size() >= 1
				 && cbExposureTime[0] != config.exposure_time)
			{
				setExposure((uint32_t)config.exposure_time, streams[0]);
			}

			if(config.exposure_mode_stream2 == 0
				 && streams.size() >= 2
				 && cbExposureTime[1] != config.exposure_time_stream2)
			{
				setExposure((uint32_t)config.exposure_time_stream2, streams[1]);
			}
		}
		else if(!clientsConnected && isCapturing)
		{
			OUT_INFO("no clients connected. stopping device...");
			if(cameraDevice->stopCapture() != royale::CameraStatus::SUCCESS)
			{
				OUT_ERROR("could not stop capture!");
				running = false;
				ros::shutdown();
			}
		}
		lockStatus.unlock();
	}

	void callbackConfig(pico_flexx_driver::pico_flexx_driverConfig &config, uint32_t level)
	{
		if(level == 0xFFFFFFFF)
		{
			return;
		}

		if(level & 0x01)
		{
			royale::Vector<royale::String> useCases;
			cameraDevice->getUseCases(useCases);
			OUT_INFO("reconfigured use case: " << FG_CYAN << useCases.at(config.use_case) << NO_COLOR);
			if(!setUseCase((size_t)config.use_case))
			{
				config.use_case = this->config.use_case;
				return;
			}
			this->config.use_case = config.use_case;

			// Need to explicitly set these parameters because of the following sequence of calls:
			// - setUseCase() above causes the driver to change the exposure times
			// - onNewExposure() is triggered, updates this->config
			// - server.updateConfig() correctly sets the new params
			// - execution returns here; without the following two lines, the values on the server
			//   would be reset to the stale values in config on return from this function
			config.exposure_time = this->config.exposure_time;
			config.exposure_time_stream2 = this->config.exposure_time_stream2;
		}

		if(level & 0x02)
		{
			OUT_INFO("reconfigured exposure_mode: " << FG_CYAN << (config.exposure_mode == 1 ? "automatic" : "manual") << NO_COLOR);

			royale::Vector<royale::StreamId> streams;
			cameraDevice->getStreams(streams);

			if(streams.size() < 1 || !setExposureMode(config.exposure_mode == 1, streams[0]))
			{
				config.exposure_mode = this->config.exposure_mode;
				return;
			}
			this->config.exposure_mode = config.exposure_mode;
		}

		if(level & 0x04)
		{
			OUT_INFO("reconfigured exposure_mode_stream2: " << FG_CYAN << (config.exposure_mode_stream2 == 1 ? "automatic" : "manual") << NO_COLOR);

			royale::Vector<royale::StreamId> streams;
			cameraDevice->getStreams(streams);

			if(streams.size() < 2 || !setExposureMode(config.exposure_mode_stream2 == 1, streams[1]))
			{
				config.exposure_mode_stream2 = this->config.exposure_mode_stream2;
				return;
			}
			this->config.exposure_mode_stream2 = config.exposure_mode_stream2;
		}

		if(level & 0x08)
		{
			OUT_INFO("reconfigured exposure_time: " << FG_CYAN << config.exposure_time << NO_COLOR);
			royale::Vector<royale::StreamId> streams;
			cameraDevice->getStreams(streams);
			royale::ExposureMode exposureMode;
			cameraDevice->getExposureMode(exposureMode);
			bool isCapturing(false);
			cameraDevice->isCapturing(isCapturing);
			if(exposureMode == royale::ExposureMode::AUTOMATIC
				 || (isCapturing && (streams.size() < 1 || !setExposure((uint32_t)config.exposure_time, streams[0]))))
			{
				config.exposure_time = this->config.exposure_time;
				return;
			}
			this->config.exposure_time = config.exposure_time;
		}

		if(level & 0x10)
		{
			OUT_INFO("reconfigured exposure_time_stream2: " << FG_CYAN << config.exposure_time_stream2 << NO_COLOR);
			royale::Vector<royale::StreamId> streams;
			cameraDevice->getStreams(streams);
			royale::ExposureMode exposureMode;
			cameraDevice->getExposureMode(exposureMode);
			bool isCapturing(false);
			cameraDevice->isCapturing(isCapturing);
			if(exposureMode == royale::ExposureMode::AUTOMATIC
				 || (isCapturing && (streams.size() < 2 || !setExposure((uint32_t)config.exposure_time_stream2, streams[1]))))
			{
				config.exposure_time_stream2 = this->config.exposure_time_stream2;
				return;
			}
			this->config.exposure_time_stream2 = config.exposure_time_stream2;
		}

		if(level & 0x20)
		{
			OUT_INFO("reconfigured max_noise: " << FG_CYAN << config.max_noise << " meters" << NO_COLOR);
			lockStatus.lock();
			this->config.max_noise = config.max_noise;
			lockStatus.unlock();
		}

		if(level & 0x40)
		{
			OUT_INFO("reconfigured range_factor: " << FG_CYAN << config.range_factor << " meters" << NO_COLOR);
			lockStatus.lock();
			this->config.range_factor = config.range_factor;
			lockStatus.unlock();
		}

		//Customization
		if(level & 0x80)
		{
			OUT_INFO("reconfigured min_depth (Customization): " << FG_MAGENTA << config.min_depth << " meters" << NO_COLOR);
			lockStatus.lock();
			this->config.min_depth = config.min_depth;
			lockStatus.unlock();
		}
		//Customization

		if(level & (0x01 | 0x02 | 0x04))
		{
			royale::Pair<uint32_t, uint32_t> limits;
			royale::Vector<royale::StreamId> streams;
			cameraDevice->getStreams(streams);

			if (streams.size() >= 1)
			{
				cameraDevice->getExposureLimits(limits, streams[0]);
				configMin.exposure_time = limits.first;
				configMax.exposure_time = limits.second;
			}
			if (streams.size() >= 2)
			{
				cameraDevice->getExposureLimits(limits, streams[1]);
				configMin.exposure_time_stream2 = limits.first;
				configMax.exposure_time_stream2 = limits.second;
			}

			server.setConfigMin(configMin);
			server.setConfigMax(configMax);
		}
	}

private:

	void init_frustrum()
	{
		// FOV -> Radians -> half
		double h_FOV_rad = 62.0 * M_PI / 180.0;
		double v_FOV_rad = 45.0 * M_PI / 180.0;
		double h_lim_2 = h_FOV_rad / 2.0;
    	double v_lim_2 = v_FOV_rad / 2.0;
		double h_res = 224.0;
		double v_res = 171.0;
		double h_res_inc  = h_FOV_rad / h_res;
		double v_res_inc  = v_FOV_rad / v_res;

		double max_range = 4.0;
		
		frustum_endpoints_x.resize(int(h_res), 0.0);
		frustum_endpoints_y.resize(int(h_res), 0.0);
		frustum_endpoints_z.resize(int(v_res), 0.0);


		// Top to Bottom
		for(int row_i = 0; row_i < v_res; ++row_i)
		{
			float dv = (row_i*v_res_inc) - v_lim_2;
			float x = float(max_range * sin(dv));
			frustum_endpoints_x[row_i] = x;
		}
		for(int col_i = 0; col_i < h_res; ++col_i)
		{
			float dh = (col_i*h_res_inc) - h_lim_2;
			float z = float(max_range * cos(dh));
			float y = float(max_range * sin(dh));
			frustum_endpoints_z[col_i] = z;
			frustum_endpoints_y[col_i] = y;
		}
		// for(int col_i = 0; col_i < v_res; ++col_i)
		// {

		// 	for(int row_i = 0; row_i < v_res; ++row_i)
		// 	{
		// 		float x = frustum_endpoints_x[col_i];
		// 		float y = frustum_endpoints_y[col_i];
		// 		float z = frustum_endpoints_z[row_i];
		// 		ROS_INFO("%f,%f,%f, sizes = %i,%i,%i", x,y,z, frustum_endpoints_x.size(), frustum_endpoints_y.size(), frustum_endpoints_z.size());
		// 	}
		// }

	}

	bool initialize()
	{
		if(running)
		{
			OUT_ERROR("driver is already running!");
			return false;
		}

		bool automaticExposure, automaticExposureStream2;
		int32_t useCase, exposureTime, exposureTimeStream2, queueSize;
		std::string sensor, baseName;
		double maxNoise, rangeFactor;

		//Customization
		double minDepth;
		//Customization

		priv_nh.param("base_name", baseName, std::string(PF_DEFAULT_NS));
		priv_nh.param("sensor", sensor, std::string(""));
		priv_nh.param("use_case", useCase, 0);
		priv_nh.param("automatic_exposure", automaticExposure, true);
		priv_nh.param("automatic_exposure", automaticExposureStream2, true);
		priv_nh.param("exposure_time", exposureTime, 1000);
		priv_nh.param("exposure_time_stream2", exposureTimeStream2, 1000);
		priv_nh.param("max_noise", maxNoise, 0.7);
		priv_nh.param("range_factor", rangeFactor, 2.0);
		priv_nh.param("queue_size", queueSize, 2);
		priv_nh.param("base_name_tf", baseNameTF, baseName);

		//Customization
		priv_nh.param("min_depth", minDepth, 0.0);
		//Customization

		OUT_INFO("parameter:" << std::endl
						 << "                 base_name: " FG_CYAN << baseName << NO_COLOR << std::endl
						 << "                    sensor: " FG_CYAN << (sensor.empty() ? "default" : sensor) << NO_COLOR << std::endl
						 << "                  use_case: " FG_CYAN << useCase << NO_COLOR << std::endl
						 << "        automatic_exposure: " FG_CYAN << (automaticExposure ? "true" : "false") << NO_COLOR << std::endl
						 << "automatic_exposure_stream2: " FG_CYAN << (automaticExposureStream2 ? "true" : "false") << NO_COLOR << std::endl
						 << "             exposure_time: " FG_CYAN << exposureTime << NO_COLOR << std::endl
						 << "     exposure_time_stream2: " FG_CYAN << exposureTimeStream2 << NO_COLOR << std::endl
						 << "                 max_noise: " FG_CYAN << maxNoise << " meters" NO_COLOR << std::endl

			 //Customization
						 << "         min_depth (Customization): " FG_MAGENTA << minDepth << " meters" NO_COLOR << std::endl
			 //Customization

						 << "              range_factor: " FG_CYAN << rangeFactor << NO_COLOR << std::endl
						 << "                queue_size: " FG_CYAN << queueSize << NO_COLOR << std::endl
						 << "              base_name_tf: " FG_CYAN << baseNameTF << NO_COLOR);

		uint32_t major, minor, patch, build;
		royale::getVersion(major, minor, patch, build);
		OUT_INFO("libroyale version: " FG_CYAN << major << '.' << minor << '.' << patch << '.' << build << NO_COLOR);

		royale::LensParameters params;
		if(!selectCamera(sensor)
			 || !setUseCase((size_t)useCase)
			 || !setExposureModeAllStreams(automaticExposure, automaticExposureStream2)
			 || !getCameraSettings(params)
			 || !createCameraInfo(params))
		{
			return false;
		}

		if(cameraDevice->registerExposureListener(this) != royale::CameraStatus::SUCCESS)
		{
			OUT_ERROR("could not register exposure listener!");
			return false;
		}


		if(cameraDevice->registerDataListener(this) != royale::CameraStatus::SUCCESS)
		{
			OUT_ERROR("could not register data listener!");
			return false;
		}

		setTopics(baseName, queueSize);

		royale::Pair<uint32_t, uint32_t> limits;
		cameraDevice->getExposureLimits(limits);
		configMin.exposure_time = limits.first;
		configMax.exposure_time = limits.second;
		server.setConfigMin(configMin);
		server.setConfigMax(configMax);

		royale::Vector<royale::String> useCases;
		cameraDevice->getUseCases(useCases);

		config.use_case = std::max(std::min(useCase, (int)useCases.size() - 1), 0);
		config.exposure_mode = automaticExposure ? 1 : 0;
		config.exposure_mode_stream2 = automaticExposureStream2 ? 1 : 0;
		config.exposure_time = std::max(std::min(exposureTime, configMax.exposure_time), configMin.exposure_time);
		config.exposure_time_stream2 = std::max(std::min(exposureTimeStream2, configMax.exposure_time_stream2), configMin.exposure_time_stream2);
		config.max_noise = std::max(std::min(maxNoise, configMax.max_noise), configMin.max_noise);
		config.range_factor = std::max(std::min(rangeFactor, configMax.range_factor), configMin.range_factor);

		//Customization
		config.min_depth = std::max(std::min(minDepth, configMax.min_depth), configMin.min_depth);
		//Customization

		server.setConfigDefault(config);

		dynamic_reconfigure::Server<pico_flexx_driver::pico_flexx_driverConfig>::CallbackType f;
		f = boost::bind(&PicoFlexx::callbackConfig, this, _1, _2);
		server.setCallback(f);

		init_frustrum();

		return true;
	}

	void setTopics(const std::string &baseName, const int32_t queueSize)
	{
		publisher.resize(2);
		ros::SubscriberStatusCallback cb = boost::bind(&PicoFlexx::callbackTopicStatus, this);

		publisher[0].resize(COUNT);
		publisher[0][CAMERA_INFO] = nh.advertise<sensor_msgs::CameraInfo>(baseName + PF_TOPIC_INFO, queueSize, cb, cb);

		//Customization
		#ifdef PUB_IMAGES
		publisher[0][MONO_8] = nh.advertise<sensor_msgs::Image>(baseName + PF_TOPIC_MONO8, queueSize, cb, cb);
		publisher[0][MONO_16] = nh.advertise<sensor_msgs::Image>(baseName + PF_TOPIC_MONO16, queueSize, cb, cb);
		publisher[0][DEPTH] = nh.advertise<sensor_msgs::Image>(baseName + PF_TOPIC_DEPTH, queueSize, cb, cb);
		publisher[0][NOISE] = nh.advertise<sensor_msgs::Image>(baseName + PF_TOPIC_NOISE, queueSize, cb, cb);
		#endif //PUB_IMAGES
		//Customization
		//Customization
		#ifdef PUB_CLOUD
		publisher[0][CLOUD] = nh.advertise<sensor_msgs::PointCloud2>(baseName + PF_TOPIC_CLOUD, queueSize, cb, cb);
		#endif //PUB_CLOUD
		//Customization

		//Customization
		points_publisher 		= nh.advertise<sensor_msgs::PointCloud2>(baseName + PF_TOPIC_CLOUD + "_reduced", queueSize, cb, cb);
		free_points_publisher 	= nh.advertise<sensor_msgs::PointCloud2>(baseName + PF_TOPIC_CLOUD + "_free", queueSize, cb, cb);
		//Customization

		publisher[1].resize(COUNT);
		publisher[1][CAMERA_INFO] = nh.advertise<sensor_msgs::CameraInfo>(baseName + "/stream2" + PF_TOPIC_INFO, queueSize, cb, cb);

		//Customization
		#ifdef PUB_IMAGES
		publisher[1][MONO_8] = nh.advertise<sensor_msgs::Image>(baseName + "/stream2" + PF_TOPIC_MONO8, queueSize, cb, cb);
		publisher[1][MONO_16] = nh.advertise<sensor_msgs::Image>(baseName + "/stream2" + PF_TOPIC_MONO16, queueSize, cb, cb);
		publisher[1][DEPTH] = nh.advertise<sensor_msgs::Image>(baseName + "/stream2" + PF_TOPIC_DEPTH, queueSize, cb, cb);
		publisher[1][NOISE] = nh.advertise<sensor_msgs::Image>(baseName + "/stream2" + PF_TOPIC_NOISE, queueSize, cb, cb);
		#endif //PUB_IMAGES
		//Customization
		//Customization
		#ifdef PUB_CLOUD
		publisher[1][CLOUD] = nh.advertise<sensor_msgs::PointCloud2>(baseName + "/stream2" + PF_TOPIC_CLOUD, queueSize, cb, cb);
		#endif //PUB_CLOUD
		//Customization

	}

	bool selectCamera(const std::string &id)
	{
		royale::String _id = id;
		royale::CameraManager manager;

		royale::Vector<royale::String> camlist = manager.getConnectedCameraList();
		if(camlist.empty())
		{
			OUT_ERROR("no cameras connected!");
			return false;
		}

		OUT_INFO("Detected " << camlist.size() << " camera(s):");

		if(id.empty())
		{
			_id = camlist[0];
		}

		int index = -1;
		for(size_t i = 0; i < camlist.size(); ++i)
		{
			if(_id == camlist[i])
			{
				index = (int)i;
				OUT_INFO("  " << i << ": " FG_CYAN << camlist[i] << FG_YELLOW " (selected)" << NO_COLOR);
			}
			else
			{
				OUT_INFO("  " << i << ": " FG_CYAN << camlist[i] << NO_COLOR);
			}
		}

		if(index < 0)
		{
			OUT_ERROR("camera with id '" << _id << "' not found!");
			return false;
		}
		cameraDevice = manager.createCamera(camlist[index]);

		if(cameraDevice == nullptr)
		{
			OUT_ERROR("cannot create camera device!");
			return false;
		}

		if(cameraDevice->initialize() != royale::CameraStatus::SUCCESS)
		{
			OUT_ERROR("cannot initialize camera device");
			return false;
		}
		return true;
	}

	bool getCameraSettings(royale::LensParameters &params)
	{
		bool ret = true;
		royale::Vector<royale::String> useCases;
		cameraDevice->getUseCases(useCases);
		royale::String useCase;
		cameraDevice->getCurrentUseCase(useCase);
		royale::ExposureMode expMode;
		cameraDevice->getExposureMode(expMode);
		royale::Pair<uint32_t, uint32_t> limits;
		cameraDevice->getExposureLimits(limits);
		royale::Vector<royale::Pair<royale::String,royale::String>> info;
		cameraDevice->getCameraInfo(info);

		royale::String cameraProperty;
		cameraDevice->getCameraName(cameraProperty);
		OUT_INFO("camera name: " FG_CYAN << cameraProperty << NO_COLOR);
		cameraDevice->getId(cameraProperty);
		OUT_INFO("camera id: " FG_CYAN << cameraProperty << NO_COLOR);
		royale::CameraAccessLevel accessLevel;
		cameraDevice->getAccessLevel(accessLevel);
		OUT_INFO("access level: " FG_CYAN "L" << (int)accessLevel+ 1 << NO_COLOR);
		OUT_INFO("exposure mode: " FG_CYAN << (expMode == royale::ExposureMode::AUTOMATIC ? "automatic" : "manual") << NO_COLOR);
		OUT_INFO("exposure limits: " FG_CYAN << limits.first << " / " << limits.second << NO_COLOR);

		OUT_INFO("camera info:");
		if(info.empty())
		{
			OUT_INFO("  no camera info available!");
		}
		else
		{
			for(size_t i = 0; i < info.size(); ++i)
			{
				OUT_INFO("  " << info[i].first << ": " FG_CYAN << info[i].second << NO_COLOR);
			}
		}

		OUT_INFO("use cases:");
		if(useCases.empty())
		{
			OUT_ERROR("  no use cases available!");
			ret = false;
		}
		else
		{
			for(size_t i = 0; i < useCases.size(); ++i)
			{
				OUT_INFO("  " << i << ": " FG_CYAN << useCases[i] << (useCases[i] == useCase ? FG_YELLOW " (selected)" : "") << NO_COLOR);
			}
		}

		if(cameraDevice->getLensParameters(params) == royale::CameraStatus::SUCCESS)
		{
			OUT_INFO("camera intrinsics:");
			uint16_t maxSensorWidth;
			cameraDevice->getMaxSensorWidth(maxSensorWidth);
			OUT_INFO("width: " FG_CYAN << maxSensorWidth << NO_COLOR);

			uint16_t maxSensorHeight;
			cameraDevice->getMaxSensorHeight(maxSensorHeight);
			OUT_INFO("height: " FG_CYAN << maxSensorHeight << NO_COLOR);
			OUT_INFO("fx: " FG_CYAN << params.focalLength.first
							 << NO_COLOR ", fy: " FG_CYAN << params.focalLength.second
							 << NO_COLOR ", cx: " FG_CYAN << params.principalPoint.first
							 << NO_COLOR ", cy: " FG_CYAN << params.principalPoint.second << NO_COLOR);
			if(params.distortionRadial.size() == 3)
			{
				OUT_INFO("k1: " FG_CYAN << params.distortionRadial[0]
								 << NO_COLOR ", k2: " FG_CYAN << params.distortionRadial[1]
								 << NO_COLOR ", p1: " FG_CYAN << params.distortionTangential.first
								 << NO_COLOR ", p2: " FG_CYAN << params.distortionTangential.second
								 << NO_COLOR ", k3: " FG_CYAN << params.distortionRadial[2] << NO_COLOR);
			}
			else
			{
				OUT_ERROR("distortion model unknown!");
				ret = false;
			}
		}
		else
		{
			OUT_ERROR("could not get lens parameter!");
			ret = false;
		}
		return ret;
	}

	bool setUseCase(const size_t idx)
	{
		royale::Vector<royale::String> useCases;
		cameraDevice->getUseCases(useCases);
		royale::String useCase;
		cameraDevice->getCurrentUseCase(useCase);

		if(useCases.empty())
		{
			OUT_ERROR("no use cases available!");
			return false;
		}

		if(idx >= useCases.size())
		{
			OUT_ERROR("use case invalid!");
			return false;
		}

		if(useCases[idx] == useCase)
		{
			OUT_INFO("use case not changed!");
			return true;
		}

		if(cameraDevice->setUseCase(useCases[idx]) != royale::CameraStatus::SUCCESS)
		{
			OUT_ERROR("could not set use case!");
			return false;
		}
		OUT_INFO("use case changed to: " FG_YELLOW << useCases[idx]);

		std::string name = royale::String::toStdString(useCases[idx]);
		size_t start, end;

		// handle MODE_9_5FPS_2000 etc.
		end = name.find("FPS");
		start = name.rfind('_', end);
		if (start != std::string::npos)
			start += 1;

		if(end == std::string::npos || start == std::string::npos)
		{
			// handle MODE_MIXED_30_5, MODE_MIXED_50_5
			start = name.find("MIXED_");
			if (start != std::string::npos)
				start += 6;
			end = name.find("_", start);
		}

		if(end == std::string::npos || start == std::string::npos)
		{
			OUT_WARN("could not extract frames per second from operation mode.");
			lockTiming.lock();
			framesPerTiming = 100;
			lockTiming.unlock();
			return true;
		}

		std::string fpsString = name.substr(start, end - start);
		if(fpsString.find_first_not_of("0123456789") != std::string::npos)
		{
			OUT_WARN("could not extract frames per second from operation mode.");
			lockTiming.lock();
			framesPerTiming = 100;
			lockTiming.unlock();
			return true;
		}

		lockTiming.lock();
		framesPerTiming = std::stoi(fpsString) * 5;
		lockTiming.unlock();
		return true;
	}

	bool setExposureModeAllStreams(const bool automatic, const bool automaticStream2)
	{
		bool success = true;
		royale::Vector<royale::StreamId> streams;
		cameraDevice->getStreams(streams);
		if (streams.size() >= 1)
			success &= setExposureMode(automatic, streams[0]);
		if (streams.size() >= 2)
			success &= setExposureMode(automaticStream2, streams[1]);
		return success;
	}

	bool setExposureMode(const bool automatic, const royale::StreamId streamId = 0)
	{
		royale::ExposureMode newMode = automatic ? royale::ExposureMode::AUTOMATIC : royale::ExposureMode::MANUAL;

		royale::ExposureMode exposureMode;
		cameraDevice->getExposureMode(exposureMode, streamId);
		if(newMode == exposureMode)
		{
			OUT_INFO("exposure mode not changed!");
			return true;
		}

		if(cameraDevice->setExposureMode(newMode, streamId) != royale::CameraStatus::SUCCESS)
		{
			OUT_ERROR("could not set operation mode!");
			return false;
		}

		OUT_INFO("exposure mode changed to: " FG_YELLOW << (automatic ? "automatic" : "manual"));
		return true;
	}

	bool setExposure(const uint32_t exposure, const royale::StreamId streamId = 0)
	{
		royale::Pair<uint32_t, uint32_t> limits;
		cameraDevice->getExposureLimits(limits, streamId);

		if(exposure < limits.first || exposure > limits.second)
		{
			OUT_ERROR("exposure outside of limits!");
			return false;
		}

		if(cameraDevice->setExposureTime(exposure, streamId) != royale::CameraStatus::SUCCESS)
		{
			OUT_ERROR("could not set exposure time!");
			return false;
		}

		OUT_INFO("exposure time changed to: " FG_YELLOW << exposure);
		return true;
	}

	bool createCameraInfo(const royale::LensParameters &params)
	{
		if(params.distortionRadial.size() != 3)
		{
			OUT_ERROR("distortion model unknown!" << params.distortionRadial.size());
			return false;
		}

		uint16_t maxSensorHeight;
		cameraDevice->getMaxSensorHeight(maxSensorHeight);
		cameraInfo.height = maxSensorHeight;

		uint16_t maxSensorWidth;
		cameraDevice->getMaxSensorWidth(maxSensorWidth);
		cameraInfo.width = maxSensorWidth;

		cameraInfo.K[0] = params.focalLength.first;
		cameraInfo.K[1] = 0;
		cameraInfo.K[2] = params.principalPoint.first;
		cameraInfo.K[3] = 0;
		cameraInfo.K[4] = params.focalLength.second;
		cameraInfo.K[5] = params.principalPoint.second;
		cameraInfo.K[6] = 0;
		cameraInfo.K[7] = 0;
		cameraInfo.K[8] = 1;

		cameraInfo.R[0] = 1;
		cameraInfo.R[1] = 0;
		cameraInfo.R[2] = 0;
		cameraInfo.R[3] = 0;
		cameraInfo.R[4] = 1;
		cameraInfo.R[5] = 0;
		cameraInfo.R[6] = 0;
		cameraInfo.R[7] = 0;
		cameraInfo.R[8] = 1;

		cameraInfo.P[0] = params.focalLength.first;
		cameraInfo.P[1] = 0;
		cameraInfo.P[2] = params.principalPoint.first;
		cameraInfo.P[3] = 0;
		cameraInfo.P[4] = 0;
		cameraInfo.P[5] = params.focalLength.second;
		cameraInfo.P[6] = params.principalPoint.second;
		cameraInfo.P[7] = 0;
		cameraInfo.P[8] = 0;
		cameraInfo.P[9] = 0;
		cameraInfo.P[10] = 1;
		cameraInfo.P[11] = 0;

		cameraInfo.distortion_model = "plumb_bob";
		cameraInfo.D.resize(5);
		cameraInfo.D[0] = params.distortionRadial[0];
		cameraInfo.D[1] = params.distortionRadial[1];
		cameraInfo.D[2] = params.distortionTangential.first;
		cameraInfo.D[3] = params.distortionTangential.second;
		cameraInfo.D[4] = params.distortionRadial[2];

		return true;
	}

	void process()
	{
		std::unique_ptr<royale::DepthData> data;
		sensor_msgs::CameraInfoPtr msgCameraInfo;
		sensor_msgs::ImagePtr msgMono8, msgMono16, msgDepth, msgNoise;
		sensor_msgs::PointCloud2Ptr msgCloud;

		//Customization
		sensor_msgs::PointCloud2Ptr msgCloudReduced;
		sensor_msgs::PointCloud2Ptr msgCloudFree;
		//Customization

		msgCameraInfo = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo);
		msgMono8 = sensor_msgs::ImagePtr(new sensor_msgs::Image);
		msgMono16 = sensor_msgs::ImagePtr(new sensor_msgs::Image);
		msgDepth = sensor_msgs::ImagePtr(new sensor_msgs::Image);
		msgNoise = sensor_msgs::ImagePtr(new sensor_msgs::Image);
		msgCloud = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2);

		//Customization
		msgCloudReduced = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2);
		msgCloudFree = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2);
		//Customization

		std::chrono::high_resolution_clock::time_point start, end;
		std::unique_lock<std::mutex> lock(lockData);
		while(running && ros::ok())
		{
			if(!cvNewData.wait_for(lock, std::chrono::milliseconds(300), [this] { return this->newData; }))
			{
				continue;
			}
			
			start = std::chrono::high_resolution_clock::now();
			this->data.swap(data);
			newData = false;
			lock.unlock();

			lockStatus.lock();
			size_t streamIndex;
			if (findStreamIndex(data->streamId, streamIndex))
			{
				extractData(*data, msgCameraInfo, msgCloud, msgCloudReduced, msgCloudFree, msgMono8, msgMono16, msgDepth, msgNoise, streamIndex);
				publish(msgCameraInfo, msgCloud, msgCloudReduced, msgCloudFree,  msgMono8, msgMono16, msgDepth, msgNoise, streamIndex);
			}
			lockStatus.unlock();

			end = std::chrono::high_resolution_clock::now();
			lockTiming.lock();
			processTime += (end - start).count();

			timings();
			lockTiming.unlock();

			lock.lock();
		}
	}

	bool findStreamIndex(const royale::StreamId streamId, size_t &streamIndex)
	{
		royale::Vector<royale::StreamId> streams;
		cameraDevice->getStreams(streams);
		auto it = std::find(streams.begin(), streams.end(), streamId);
		if (it == streams.end())
		{
			OUT_ERROR("invalid stream ID!");
			return false;
		}
		streamIndex = std::distance(streams.begin(), it);
		return true;
	}

	void extractData(const royale::DepthData &data, sensor_msgs::CameraInfoPtr &msgCameraInfo, sensor_msgs::PointCloud2Ptr &msgCloud, 
										sensor_msgs::PointCloud2Ptr &msgCloudReduced,sensor_msgs::PointCloud2Ptr &msgCloudFree,
									 sensor_msgs::ImagePtr &msgMono8, sensor_msgs::ImagePtr &msgMono16, sensor_msgs::ImagePtr &msgDepth, sensor_msgs::ImagePtr &msgNoise,
									 size_t streamIndex = 0) const
	{
		std_msgs::Header header;
		header.frame_id = baseNameTF + PF_TF_OPT_FRAME;
		header.seq = 0;
		header.stamp.fromNSec(std::chrono::duration_cast<std::chrono::nanoseconds>(data.timeStamp).count());

		if(!pub_status)
		{
			return;
		}
		//Customization
		const unsigned int EXTRASTEP = 3*sizeof(float) + 1*sizeof(uint16_t);

		msgCloudReduced->header = header;
		msgCloudReduced->height = data.height;
		msgCloudReduced->width = data.width;
		msgCloudReduced->is_bigendian = false;
		msgCloudReduced->is_dense = false;
		msgCloudReduced->point_step = (uint32_t)(EXTRASTEP);
		msgCloudReduced->row_step = (uint32_t)(EXTRASTEP * data.width);
		msgCloudReduced->fields.resize(4);
		msgCloudReduced->fields[0].name = "x";
		msgCloudReduced->fields[0].offset = 0;
		msgCloudReduced->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
		msgCloudReduced->fields[0].count = 1;
		msgCloudReduced->fields[1].name = "y";
		msgCloudReduced->fields[1].offset = msgCloudReduced->fields[0].offset + (uint32_t)sizeof(float);
		msgCloudReduced->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
		msgCloudReduced->fields[1].count = 1;
		msgCloudReduced->fields[2].name = "z";
		msgCloudReduced->fields[2].offset = msgCloudReduced->fields[1].offset + (uint32_t)sizeof(float);
		msgCloudReduced->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
		msgCloudReduced->fields[2].count = 1;
		msgCloudReduced->fields[3].name = "intensity";
		msgCloudReduced->fields[3].offset = msgCloudReduced->fields[2].offset + (uint32_t)sizeof(float);
		msgCloudReduced->fields[3].datatype = sensor_msgs::PointField::UINT16;
		msgCloudReduced->fields[3].count = 1;
		msgCloudReduced->data.resize(EXTRASTEP * data.points.size());

		msgCloudFree->header = header;
		msgCloudFree->height = data.height;
		msgCloudFree->width = data.width;
		msgCloudFree->is_bigendian = false;
		msgCloudFree->is_dense = false;
		msgCloudFree->point_step = (uint32_t)(EXTRASTEP);
		msgCloudFree->row_step = (uint32_t)(EXTRASTEP * data.width);
		msgCloudFree->fields.resize(4);
		msgCloudFree->fields[0].name = "x";
		msgCloudFree->fields[0].offset = 0;
		msgCloudFree->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
		msgCloudFree->fields[0].count = 1;
		msgCloudFree->fields[1].name = "y";
		msgCloudFree->fields[1].offset = msgCloudFree->fields[0].offset + (uint32_t)sizeof(float);
		msgCloudFree->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
		msgCloudFree->fields[1].count = 1;
		msgCloudFree->fields[2].name = "z";
		msgCloudFree->fields[2].offset = msgCloudFree->fields[1].offset + (uint32_t)sizeof(float);
		msgCloudFree->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
		msgCloudFree->fields[2].count = 1;
		msgCloudFree->fields[3].name = "intensity";
		msgCloudFree->fields[3].offset = msgCloudFree->fields[2].offset + (uint32_t)sizeof(float);
		msgCloudFree->fields[3].datatype = sensor_msgs::PointField::UINT16;
		msgCloudFree->fields[3].count = 1;
		msgCloudFree->data.resize(EXTRASTEP * data.points.size());

		const float invalid = std::numeric_limits<float>::quiet_NaN();
		const float maxNoise = (float)config.max_noise;

		const float minDepth = 0.1; //(float)config.min_depth;

		const royale::DepthPoint *itI = &data.points[0];

		unsigned int extraOffset = 0;
		unsigned int freeOffset = 0;
		unsigned int free_counter = 0;
		unsigned int noise_counter = 0;
		// ROS_INFO("H: %i, W: %i", data.height, data.width);
		for(size_t i = 0; i < data.points.size(); ++i, ++itI, extraOffset += EXTRASTEP, freeOffset += EXTRASTEP)
		{
			unsigned int row = i / data.width;
			unsigned int col = i % data.width;
			if(itI->noise < maxNoise && itI->z >= minDepth &&  itI->depthConfidence != 0.0)
			{

				memcpy(&msgCloudReduced->data[extraOffset], &itI->x, sizeof(float));
				memcpy(&msgCloudReduced->data[extraOffset + 4], &itI->y, sizeof(float));
				memcpy(&msgCloudReduced->data[extraOffset + 8], &itI->z, sizeof(float));
				memcpy(&msgCloudReduced->data[extraOffset + 12], &itI->grayValue, sizeof(uint16_t));
			}
			else if(itI->noise >= maxNoise)
			{
				noise_counter++;
			}
			else if(itI->depthConfidence == 0.0 || itI->z == 0.0)
			{
				int crop = 45;
				if(row < crop && col < crop ||
					row < crop && col > data.width-crop ||
					row > data.height-crop && col > data.width-crop ||
					row > data.height-crop && col < crop)
				{
					continue;
				}
				else
				{
					float y = frustum_endpoints_x[row];
					float x = frustum_endpoints_y[col];
					float z = frustum_endpoints_z[col];
					uint16_t g = 0;
					//ROS_INFO("FREE POINT : %f,%f,%f, ROW/COL = %i,%i",x,y,z, row, col);
					free_counter++;
					memcpy(&msgCloudFree->data[freeOffset],      &x, sizeof(float));
					memcpy(&msgCloudFree->data[freeOffset + 4],  &y, sizeof(float));
					memcpy(&msgCloudFree->data[freeOffset + 8],  &z, sizeof(float));
					memcpy(&msgCloudFree->data[freeOffset + 12], &g, sizeof(uint16_t));
				}
			}
		}
		// ROS_INFO("FOUND %i FREE POINTS, NOISY POINTS: %i", free_counter, noise_counter);
	}

	void publish(sensor_msgs::CameraInfoPtr &msgCameraInfo, sensor_msgs::PointCloud2Ptr &msgCloud, 
								sensor_msgs::PointCloud2Ptr &msgCloudReduced, sensor_msgs::PointCloud2Ptr &msgCloudFree,
							 sensor_msgs::ImagePtr &msgMono8, sensor_msgs::ImagePtr &msgMono16,
							 sensor_msgs::ImagePtr &msgDepth, sensor_msgs::ImagePtr &msgNoise,
							 size_t streamIndex = 0) const
	{
		if(status[streamIndex][CAMERA_INFO])
		{
			publisher[streamIndex][CAMERA_INFO].publish(msgCameraInfo);
			msgCameraInfo = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo);
		}
		if (pub_status){
			points_publisher.publish(msgCloudReduced);
			msgCloudReduced = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2);
			free_points_publisher.publish(msgCloudFree);
			msgCloudFree = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2);
		}

	}

	void timings()
	{
		std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();

		if(!frame)
		{
			startTime = now;
		}
		else if(frame % framesPerTiming == 0)
		{
			double timePerFrame, framesPerSecond, avgDelay;

			timePerFrame = (double)(processTime / framesPerTiming) / 1000000.0;
			framesPerSecond = (double)framesPerTiming / ((double)(now - startTime).count() / 1000000000.0);
			avgDelay = ((double)delayReceived / (double)framesPerTiming) / 1000000.0;

			processTime = 0;
			startTime = now;
			delayReceived = 0;
			OUT_INFO("processing: " FG_YELLOW "~" << std::setprecision(4) << timePerFrame << " ms." NO_COLOR
							 " fps: " FG_YELLOW "~" << framesPerSecond << " Hz" NO_COLOR
							 " delay: " FG_YELLOW "~" << avgDelay << " ms." NO_COLOR);
		}
		++frame;
	}
};

class PicoFlexxNodelet : public nodelet::Nodelet
{
private:
	PicoFlexx *picoFlexx;

public:
	PicoFlexxNodelet() : Nodelet(), picoFlexx(NULL)
	{
	}

	~PicoFlexxNodelet()
	{
		if(picoFlexx)
		{
			picoFlexx->stop();
			delete picoFlexx;
		}
	}

	virtual void onInit()
	{
		picoFlexx = new PicoFlexx(getNodeHandle(), getPrivateNodeHandle());
		picoFlexx->start();
	}
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(PicoFlexxNodelet, nodelet::Nodelet)

int main(int argc, char **argv)
{
#if EXTENDED_OUTPUT
	ROSCONSOLE_AUTOINIT;
	if(!getenv("ROSCONSOLE_FORMAT"))
	{
		ros::console::g_formatter.tokens_.clear();
		ros::console::g_formatter.init("[${severity}] ${message}");
	}
#endif

	ros::init(argc, argv, PF_DEFAULT_NS);

	PicoFlexx picoFlexx;
	picoFlexx.start();
	ros::spin();
	picoFlexx.stop();
	return 0;
}
