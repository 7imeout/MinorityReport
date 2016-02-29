// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

///////////////////////////////////////////////////////
// librealsense tutorial #3 - Point cloud generation //
///////////////////////////////////////////////////////

// First include the librealsense C++ header file
#include <librealsense/rs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdio>
#include <iostream>

// Also include GLFW to allow for graphical display
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

double yaw, pitch, lastX, lastY; int ml;
static void on_mouse_button(GLFWwindow * win, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT) ml = action == GLFW_PRESS;
}
static double clamp(double val, double lo, double hi) { return val < lo ? lo : val > hi ? hi : val; }
static void on_cursor_pos(GLFWwindow * win, double x, double y)
{
	if (ml)
	{
		yaw = clamp(yaw - (x - lastX), -120, 120);
		pitch = clamp(pitch + (y - lastY), -80, 80);
	}
	lastX = x;
	lastY = y;
}

int runWindow(GLFWwindow * win, const uint16_t * depth_image, const uint8_t * color_image,
	rs::intrinsics depth_intrin, rs::extrinsics depth_to_color, rs::intrinsics color_intrin, float scale) {


	// Set up a perspective transform in a space that we can rotate by clicking and dragging the mouse
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, (float)1280 / 960, 0.01f, 20.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);
	glTranslatef(0, 0, +0.5f);
	glRotated(pitch, 1, 0, 0);
	glRotated(yaw, 0, 1, 0);
	glTranslatef(0, 0, -0.5f);

	// We will render our depth data as a set of points in 3D space
	glPointSize(2);
	glEnable(GL_DEPTH_TEST);
	glBegin(GL_POINTS);

	for (int dy = 0; dy<depth_intrin.height; ++dy)
	{
		for (int dx = 0; dx<depth_intrin.width; ++dx)
		{
			// Retrieve the 16-bit depth value and map it into a depth in meters
			uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
			float depth_in_meters = depth_value * scale;

			// Skip over pixels with a depth value of zero, which is used to indicate no data
			if (depth_value == 0) continue;

			// Map from pixel coordinates in the depth image to pixel coordinates in the color image
			rs::float2 depth_pixel = { (float)dx, (float)dy };
			rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
			rs::float3 color_point = depth_to_color.transform(depth_point);
			rs::float2 color_pixel = color_intrin.project(color_point);

			// Use the color from the nearest color pixel, or pure white if this point falls outside the color image
			const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
			if (cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height)
			{
				glColor3ub(255, 255, 255);
			}
			else
			{
				glColor3ubv(color_image + (cy * color_intrin.width + cx) * 3);
			}

			// Emit a vertex at the 3D location of this depth pixel
			glVertex3f(depth_point.x, depth_point.y, depth_point.z);
		}
	}
	glEnd();

	glfwSwapBuffers(win);

	return 0;
}

bool runCamera(rs::device * camera, GLFWwindow * win, int cameraID) {
	//camera->enable_stream(rs::stream::depth, rs::preset::best_quality);
	camera->wait_for_frames();
	int32_t numpoints = 0;

	const int STACK_SIZE = 10;
	static int32_t prevPtsStack[STACK_SIZE] = {};
	static int32_t * stackHead = prevPtsStack;
	static int iterationCtr = 0;

	// Retrieve our images
	const uint16_t * depth_image = (const uint16_t *)camera->get_frame_data(rs::stream::depth);
	const uint8_t * color_image = (const uint8_t *)camera->get_frame_data(rs::stream::color);


	// Retrieve camera parameters for mapping between depth and color
	rs::intrinsics depth_intrin = camera->get_stream_intrinsics(rs::stream::depth);
	rs::extrinsics depth_to_color = camera->get_extrinsics(rs::stream::depth, rs::stream::color);
	rs::intrinsics color_intrin = camera->get_stream_intrinsics(rs::stream::color);
	float scale = camera->get_depth_scale();
	float maxX = 0, maxY = 0, maxZ = 0;
	for (int dy = 0; dy < depth_intrin.height; dy += 2)
	{
		for (int dx = 0; dx < depth_intrin.width; dx += 2)
		{
			// Retrieve the 16-bit depth value and map it into a depth in meters
			uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
			float depth_in_meters = depth_value * scale;

			// Skip over pixels with a depth value of zero, which is used to indicate no data
			if (depth_value == 0) continue;

			rs::float2 depth_pixel = { (float)dx, (float)dy };
			rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);

			rs::float3 color_point = depth_to_color.transform(depth_point);
			rs::float2 color_pixel = color_intrin.project(color_point);

			// Use the color from the nearest color pixel, or pure white if this point falls outside the color image
			const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);

			// Check for green to get some kind of helpful data

			float x = depth_point.x;
			float y = depth_point.y;
			float z = depth_point.z;
			

			if (z < .5)
				numpoints++;

			//setup maxes
			if (maxX < x)
				maxX = x;
			if (maxY < y)
				maxY = y;
			if (maxZ < z)
				maxZ = z;

			//printf("x: %.2f, y: %.2f, z: %.2f | ", x, y, z);
		}
		//printf("\n");
	}
	//printf("Camera %d - max x: %.2f, max y: %.2f, max z: %.2f \n", number, maxX, maxY, maxZ);
	//printf("Camera %d - number points  %d\n", number, numpoints);

	if (numpoints > 2500)
	{
		prevPtsStack[iterationCtr % STACK_SIZE] = numpoints;
		stackHead = &prevPtsStack[iterationCtr % STACK_SIZE];

		int avg, sum = 0;
		for (int i = 0; i < STACK_SIZE; i++) sum += prevPtsStack[i];
		avg = sum / STACK_SIZE;

		printf("Camera %d - number points  %d, prev pts %d, recent ctr %d, sum %d, avg %d, diff %d\n", 
			cameraID, numpoints, *stackHead, iterationCtr, sum, avg, numpoints - avg);
		runWindow(win, depth_image, color_image, depth_intrin, depth_to_color, color_intrin, scale);

		if (numpoints > avg + 2000) printf("\n   *** PRIMARY CLICK GESTURE DETECTED @ %d *** \n\n", iterationCtr);
		if (numpoints < avg - 1250) printf("\n   *** SECONDARY CLICK GESTURE DETECTED @ %d *** \n\n", iterationCtr);
		
		iterationCtr++;
		return true;
	}

	//camera->disable_stream(rs::stream::depth);

	//camera->set_option(rs::option::f200_laser_power, 0);
	return false;
}

int main() try
{
	// Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
	rs::log_to_console(rs::log_severity::warn);
	//rs::log_to_file(rs::log_severity::debug, "librealsense.log");

	// Create a context object. This object owns the handles to all connected realsense devices.
	rs::context ctx;
	printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
	if (ctx.get_device_count() == 0) return EXIT_FAILURE;

	// This tutorial will access only a single device, but it is trivial to extend to multiple devices
	rs::device * dev = ctx.get_device(0);
	rs::device * dev2 = ctx.get_device(1);
	printf("\nUsing device 0, an %s\n", dev->get_name());
	printf("    Serial number: %s\n", dev->get_serial());
	printf("    Firmware version: %s\n", dev->get_firmware_version());

	// Configure depth and color to run with the device's preferred settings
	dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
	dev->enable_stream(rs::stream::color, rs::preset::best_quality);
	dev->start();

	if (ctx.get_device_count() > 1)
	{
		dev2 = ctx.get_device(1);
		// Configure depth and color to run with the device's preferred settings
		dev2->enable_stream(rs::stream::depth, rs::preset::best_quality);
		dev2->enable_stream(rs::stream::color, rs::preset::best_quality);
		dev2->start();
		dev2->set_option(rs::option::f200_laser_power, 0);
	}

	// Open a GLFW window to display our output
	glfwInit();
	GLFWwindow * win = glfwCreateWindow(1280, 960, "librealsense tutorial #3", nullptr, nullptr);
	//GLFWwindow * win2 = glfwCreateWindow(1280, 960, "librealsense tutorial #3", nullptr, nullptr);
	glfwSetCursorPosCallback(win, on_cursor_pos);
	glfwSetMouseButtonCallback(win, on_mouse_button);
	glfwMakeContextCurrent(win);
	while (!glfwWindowShouldClose(win))
	{

		// Wait for new frame data
		glfwPollEvents();

		bool hasobj = runCamera(dev, win, 0);

		if (ctx.get_device_count() > 1)
		{
			hasobj |= runCamera(dev2, win, 1);
		}

		if (!hasobj) {
			static int time = 0;
			const int SWITCH = 10;
			time++;
			if (time == SWITCH)
			{
				//dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
				//dev2->disable_stream(rs::stream::depth);
				dev->set_option(rs::option::f200_laser_power, 0);
				dev2->set_option(rs::option::f200_laser_power, 15);
			}
			else if (time == 2 * SWITCH)
			{
				//dev->disable_stream(rs::stream::depth);
				//dev2->enable_stream(rs::stream::depth, rs::preset::best_quality);
				dev->set_option(rs::option::f200_laser_power, 15);
				dev2->set_option(rs::option::f200_laser_power, 0);
				time = 0;
			}
		}

		char temp = 'x';
		//while (temp != '\n')
		//	std::cin.get(temp);
	}

	return EXIT_SUCCESS;
}
catch (const rs::error & e)
{
	// Method calls against librealsense objects may throw exceptions of type rs::error
	printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
	printf("    %s\n", e.what());
	return EXIT_FAILURE;
}
