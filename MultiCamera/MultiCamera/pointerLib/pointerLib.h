// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once
#include <librealsense/rs.hpp>
#include <vector>
#include <opencv2\opencv.hpp>
struct state {
	double yaw, pitch, lastX, lastY;
	bool ml;
	std::vector<rs::stream> tex_streams;
	float depth_scale;
	rs::extrinsics extrin;
	rs::intrinsics depth_intrin;
	rs::intrinsics tex_intrin;
	bool identical;
	int index;
	rs::device * dev;
};

// use dll for all the camera activity.
extern "C" __declspec(dllexport) state *initializePointerLib();
extern "C" __declspec(dllexport) bool pointerNextFrame(int &x, int &y, int &z);
