// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include <librealsense/rs.hpp>
#include "example.hpp"
#include <chrono>
#include <vector>
#include <sstream>
#include <iostream>
#include <algorithm>
#include "pointerLib.h"
#include "pointerLib.h"

static rs::context ctx;
static state app_state;
static int frames = 0; 
static float nexttime = 0, fps = 0;

static std::chrono::steady_clock::time_point t0;
 
extern "C" __declspec(dllexport)
state *initializePointerLib()
{
	if (ctx.get_device_count() > 0)
	{
		static rs::device & dev = *ctx.get_device(0);

		int inputWidth = 320, inputHeight = 240, frameRate = 60;
		dev.enable_stream(rs::stream::color, inputWidth, inputHeight, rs::format::rgb8, frameRate);
		dev.enable_stream(rs::stream::depth, inputWidth, inputHeight, rs::format::z16, frameRate);
		dev.start();

		// some placeholders were added for intrinsics and extrinsics.
		state initState = { 0, 0, 0, 0, false,{ rs::stream::color, rs::stream::depth, rs::stream::infrared }, dev.get_depth_scale(),
			dev.get_extrinsics(rs::stream::depth, rs::stream::color), dev.get_stream_intrinsics(rs::stream::depth),
			dev.get_stream_intrinsics(rs::stream::depth), 0, 0, &dev };
		app_state = initState;
		auto t0 = std::chrono::high_resolution_clock::now();
		return &app_state;
	}
	return 0;
}

extern "C"  __declspec(dllexport)
bool pointerNextFrame(int &xInOut, int &yInOut, int &zInOut)
{
	rs::device & dev = *app_state.dev;
	if (dev.is_streaming()) dev.wait_for_frames();

	auto t1 = std::chrono::high_resolution_clock::now();
	nexttime += std::chrono::duration<float>(t1 - t0).count();
	t0 = t1;
	++frames;
	if (nexttime > 0.5f)
	{
		fps = frames / nexttime;
		frames = 0;
		nexttime = 0;
	}

	const rs::stream tex_stream = app_state.tex_streams[app_state.index];
	app_state.depth_scale = dev.get_depth_scale();
	app_state.extrin = dev.get_extrinsics(rs::stream::depth, tex_stream);
	app_state.depth_intrin = dev.get_stream_intrinsics(rs::stream::depth);
	app_state.tex_intrin = dev.get_stream_intrinsics(tex_stream);
	app_state.identical = app_state.depth_intrin == app_state.tex_intrin && app_state.extrin.is_identity();

	// setup the OpenCV Mat structures
	cv::Mat depth16(app_state.depth_intrin.height, app_state.depth_intrin.width, CV_16U, (uchar *)dev.get_frame_data(rs::stream::depth));
	
	rs::intrinsics color_intrin = dev.get_stream_intrinsics(rs::stream::color);
	cv::Mat rgb(color_intrin.height, color_intrin.width, CV_8UC3, (uchar *)dev.get_frame_data(rs::stream::color));

	// ignore depth greater than 800 mm's.
	depth16.setTo(10000, depth16 > 800);
	depth16.setTo(10000, depth16 == 0);
	cv::Mat depth8u = depth16 < 800;

	cv::Point handPoint(0, 0);
	for (int y = 0; y < depth8u.rows; ++y)
	{
		uchar *d = depth8u.row(y).ptr();
		for (int x = 0; x < depth8u.cols; ++x)
		{
			if (d[x])
			{
				int floodCount = cv::floodFill(depth8u, cv::Point(x, y), 255);
				// we have found the top most point
				if (floodCount > 100)
				{
					handPoint = cv::Point(x, y);
					break;
				}
			}
		}
		if (handPoint != cv::Point(0, 0)) break;
	}

	if (handPoint != cv::Point(0, 0))
		cv::circle(depth8u, handPoint, 10, 128, cv::FILLED);
	imshow("depth8u", depth8u);
	cv::cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);
	imshow("rgb", rgb);
	xInOut = handPoint.x;
	yInOut = handPoint.y;
	zInOut = 0; // not using this yet.
	if (handPoint == cv::Point(0, 0)) return false;
	return true;
}