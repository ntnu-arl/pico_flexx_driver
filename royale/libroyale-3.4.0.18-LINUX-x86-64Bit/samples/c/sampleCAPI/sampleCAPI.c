/****************************************************************************\
* Copyright (C) 2016 Infineon Technologies
*
* THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
* PARTICULAR PURPOSE.
*
\****************************************************************************/

// C API version can now be selected by defining ROYALE_C_API_VERSION
// to an appropriate value before including any of the C API headers.
// See documentation in CAPIVersion.h.

#include <CameraManagerCAPI.h>
#include <CameraDeviceCAPI.h>
#include <DepthDataCAPI.h>
#include <DataStructuresCAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sample_utils/PlatformResourcesCAPI.h>

#ifdef _WINDOWS
#include <Windows.h>
#define SAMPLE_SLEEP_MS(x)        Sleep(x)
#else
#include <unistd.h>
#define SAMPLE_SLEEP_MS(x)        usleep((x)*1000)
#endif

void depth_data_callback (royale_depth_data *data)
{
    printf ("============================================================\n");
    printf ("Got data frame.\n");
    printf ("%dx%d depth image for stream %x.\n", data->width, data->height, data->stream_id);

    uint32_t center_index = (data->nr_points / 2) - 1;

    royale_depth_point point = data->points[center_index];
    printf ("center: X: %f, Y: %f, Z: %f, gray: %d,\nconfidence: %d, noise: %f\n",
            point.x, point.y, point.z, point.gray_value, point.depth_confidence, point.noise);
}

void print_error_message (const char *message, royale_camera_status error)
{
    char *error_string = royale_status_get_error_string (error);
    printf ("%s: %s\n", message, error_string);
    royale_free_string (error_string);
}

royale_camera_handle create_camera_device()
{
    royale_cam_manager_hnd man_hnd = royale_camera_manager_create();

    // get connected cameras
    uint32_t nr_cameras = 0;
    char **camera_list = royale_camera_manager_get_connected_cameras (man_hnd, &nr_cameras);

    printf ("Found %d camera(s):\n", nr_cameras);

    if (nr_cameras == 0)
    {
        royale_camera_manager_destroy (man_hnd);
        return ROYALE_NO_INSTANCE_CREATED;
    }

    uint32_t i = 0;
    for (; i < nr_cameras; i++)
    {
        printf ("%s\n", camera_list[i]);
    }

    printf ("\nTrying to open first camera in list.\n");
    royale_camera_handle cam_hnd = royale_camera_manager_create_camera (man_hnd, camera_list[0]);

    // free C string array created in royale_camera_manager_get_connected_cameras()
    royale_free_string_array (camera_list, nr_cameras);

    //destroy manager, not needed anymore
    royale_camera_manager_destroy (man_hnd);

    return cam_hnd;
}

int start_camera_actions (royale_camera_handle cam_hnd)
{
    //initialize camera device
    royale_camera_status status = royale_camera_device_initialize (cam_hnd);

    if (ROYALE_STATUS_SUCCESS != status)
    {
        print_error_message ("Failed to initialize camera", status);
        return -1;
    }

    //get some basic information about the device
    char *name;
    char *id;

    status = royale_camera_device_get_camera_name (cam_hnd, &name);
    if (ROYALE_STATUS_SUCCESS != status)
    {
        print_error_message ("Failed to get camera name", status);
        return -1;
    }

    status = royale_camera_device_get_id (cam_hnd, &id);
    if (ROYALE_STATUS_SUCCESS != status)
    {
        print_error_message ("Failed to get camera id", status);
        return -1;
    }
    printf ("Device found: %s: %s\n", name, id);

    royale_free_string (id);
    royale_free_string (name);

    // have a look, which use cases are available
    char **use_cases;
    uint32_t nr_use_cases;
    status = royale_camera_device_get_use_cases (cam_hnd, &use_cases, &nr_use_cases);

    if (ROYALE_STATUS_SUCCESS != status)
    {
        print_error_message ("Failed to get available use cases", status);
        return -1;
    }

    printf ("%d use cases available for this device:\n", nr_use_cases);
    uint32_t i = 0;
    for (; i < nr_use_cases; i++)
    {
        printf ("    %s\n", use_cases[i]);
    }
    royale_free_string_array (use_cases, nr_use_cases);

    // register listener for data
    status = royale_camera_device_register_data_listener (cam_hnd, &depth_data_callback);

    if (ROYALE_STATUS_SUCCESS != status)
    {
        print_error_message ("Failed to register data listener", status);
        return -1;
    }

    printf ("starting to capture for 2 seconds.\n\n");
    status = royale_camera_device_start_capture (cam_hnd);

    if (ROYALE_STATUS_SUCCESS != status)
    {
        print_error_message ("Failed to start capture", status);
        return -1;
    }

    SAMPLE_SLEEP_MS (2000);

    status = royale_camera_device_unregister_data_listener (cam_hnd);

    if (ROYALE_STATUS_SUCCESS != status)
    {
        print_error_message ("Failed to unregister data listener", status);
        return -1;
    }

    printf ("\nstopping capture.\n\n");
    status = royale_camera_device_stop_capture (cam_hnd);

    if (ROYALE_STATUS_SUCCESS != status)
    {
        print_error_message ("Failed to stop capture", status);
        return -1;
    }

    printf ("done.\n");
    return 0;
}

int main (int argc, char **argv)
{
    royale_camera_status status = royale_platform_resources_initialize();
    if (ROYALE_STATUS_SUCCESS != status)
    {
        print_error_message ("Failed to initialize platform resources: %s", status);
        return -1;
    }

    printf ("============================================================\n");
    printf ("|                    royale C API sample                   |\n");
    printf ("============================================================\n\n");

    royale_camera_handle cam_hnd = create_camera_device();

    if (cam_hnd == ROYALE_NO_INSTANCE_CREATED)
    {
        printf ("Failed to open camera.\n");
        royale_platform_resources_uninitialize();
        return -1;
    }

    printf ("Camera opened.\n");

    int ret_val = start_camera_actions (cam_hnd);
    if (ret_val != 0)
    {
        printf ("Error starting camera\n");
    }

    // destroy camera device instance and cleanup memory
    // this is mandatory before program exit
    royale_camera_device_destroy (cam_hnd);
    royale_platform_resources_uninitialize();

    return ret_val;
}
