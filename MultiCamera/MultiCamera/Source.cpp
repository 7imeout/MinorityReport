// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

///////////////////////////////////////////////////////
// librealsense tutorial #3 - Point cloud generation //
///////////////////////////////////////////////////////

// First include the librealsense C++ header file
#include <librealsense/rs.hpp>
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

int runCamera(rs::device * camera, GLFWwindow * win, int number) {

	//camera->enable_stream(rs::stream::depth, rs::preset::best_quality);
	camera->wait_for_frames();
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

			//Check for green to get some kind of helpful data

			float x = depth_point.x;
			float y = depth_point.y;
			float z = depth_point.z;
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
	printf("Camera %d - max x: %.2f, max y: %.2f, max z: %.2f \n", number, maxX, maxY, maxZ);

	if (number == 0)
		runWindow(win, depth_image, color_image, depth_intrin, depth_to_color, color_intrin, scale);

	//camera->disable_stream(rs::stream::depth);
	return 0;
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

		runCamera(dev, win, 0);

		if (ctx.get_device_count() > 1)
		{
			runCamera(dev2, win, 1);
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
