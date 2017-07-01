#include <royale.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <sample_utils/PlatformResources.hpp>

using namespace royale;
using namespace sample_utils;
using namespace std;
using namespace cv;

class MyListener : public IDepthDataListener
{
    void onNewData (const DepthData *data)
    {
        // this callback function will be called for every new
        // depth frame

        // create two images which will be filled afterwards
        // each image containing one 32Bit channel
        zImage.create (Size (data->width, data->height), CV_32FC1);
        grayImage.create (Size (data->width, data->height), CV_32FC1);

        // set the image to zero
        zImage = Scalar::all (0);
        grayImage = Scalar::all (0);

        int k = 0;
        for (int y = 0; y < zImage.rows; y++)
        {
            float *zRowPtr = zImage.ptr<float> (y);
            float *grayRowPtr = grayImage.ptr<float> (y);
            for (int x = 0; x < zImage.cols; x++, k++)
            {
                auto curPoint = data->points.at (k);
                if (curPoint.depthConfidence > 0)
                {
                    // if the point is valid
                    zRowPtr[x] = curPoint.z;
                    grayRowPtr[x] = curPoint.grayValue;
                }
            }
        }

        // create images to store the 8Bit version (some OpenCV
        // functions may only work on 8Bit images)
        zImage8.create (Size (data->width, data->height), CV_8UC1);
        grayImage8.create (Size (data->width, data->height), CV_8UC1);

        // this normalizes the images from min/max to 0/255
        normalize (zImage, zImage8, 0, 255, NORM_MINMAX, CV_8UC1);
        normalize (grayImage, grayImage8, 0, 255, NORM_MINMAX, CV_8UC1);

        // scale and display the depth image
        scaledZImage.create (Size (data->width * 4, data->height * 4), CV_8UC1);
        resize (zImage8, scaledZImage, scaledZImage.size());

        imshow ("Depth", scaledZImage);

        // scale and display the gray image
        scaledGrayImage.create (Size (data->width * 4, data->height * 4), CV_8UC1);
        resize (grayImage8, scaledGrayImage, scaledGrayImage.size());

        imshow ("Gray", scaledGrayImage);
    }

private:

    // define images for depth and gray
    // and for their 8Bit and scaled versions
    Mat zImage, zImage8, scaledZImage;
    Mat grayImage, grayImage8, scaledGrayImage;

};

int main (int argc, char *argv[])
{
    // Windows requires that the application allocate these, not the DLL.
    PlatformResources resources;

    // This is the data listener which will receive callbacks.  It's declared
    // before the cameraDevice so that, if this function exits with a 'return'
    // statement while the camera is still capturing, it will still be in scope
    // until the cameraDevice's destructor implicitly de-registers the listener.
    MyListener listener;

    // this represents the main camera device object
    std::unique_ptr<ICameraDevice> cameraDevice;

    // the camera manager will query for a connected camera
    {
        CameraManager manager;

        // check the number of arguments
        if (argc > 1)
        {
            // if the program was called with an argument try to open this as a file
            cout << "Trying to open : " << argv[1] << endl;
            cameraDevice = manager.createCamera (argv[1]);
        }
        else
        {
            // if no argument was given try to open the first connected camera
            royale::Vector<royale::String> camlist (manager.getConnectedCameraList());
            cout << "Detected " << camlist.size() << " camera(s)." << endl;

            if (!camlist.empty())
            {
                cameraDevice = manager.createCamera (camlist[0]);
            }
            else
            {
                cerr << "No suitable camera device detected." << endl
                     << "Please make sure that a supported camera is plugged in, all drivers are "
                     << "installed, and you have proper USB permission" << endl;
                return 1;
            }

            camlist.clear();
        }
    }
    // the camera device is now available and CameraManager can be deallocated here

    if (cameraDevice == nullptr)
    {
        // no cameraDevice available
        if (argc > 1)
        {
            cerr << "Could not open " << argv[1] << endl;
            return 1;
        }
        else
        {
            cerr << "Cannot create the camera device" << endl;
            return 1;
        }
    }

    // IMPORTANT: call the initialize method before working with the camera device
    auto status = cameraDevice->initialize();
    if (status != CameraStatus::SUCCESS)
    {
        cerr << "Cannot initialize the camera device, error string : " << getErrorString (status) << endl;
        return 1;
    }

    // register a data listener
    if (cameraDevice->registerDataListener (&listener) != CameraStatus::SUCCESS)
    {
        cerr << "Error registering data listener" << endl;
        return 1;
    }

    // create two windows
    namedWindow ("Depth", WINDOW_AUTOSIZE);
    namedWindow ("Gray", WINDOW_AUTOSIZE);

    // start capture mode
    if (cameraDevice->startCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error starting the capturing" << endl;
        return 1;
    }

    // wait until a key is pressed
    waitKey (0);

    // stop capture mode
    if (cameraDevice->stopCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error stopping the capturing" << endl;
        return 1;
    }

    return 0;
}
