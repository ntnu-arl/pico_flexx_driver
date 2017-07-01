#include <royale.hpp>
#include <iostream>
#include <thread>
#include <chrono>

#include <sample_utils/PlatformResources.hpp>

using namespace royale;
using namespace sample_utils;
using namespace std;

class MyListener : public IDepthDataListener
{
    void onNewData (const DepthData *data)
    {
        /* Demonstration of how to retrieve exposureTimes
         * There might be different ExposureTimes per RawFrameSet resulting in a vector of
         * exposureTimes, while however the last one is fixed and purely provided for further
         * reference.
         */
        auto sampleVector (data->exposureTimes);

        if (sampleVector.size() > 0)
        {
            cout << "ExposureTimes #1: ";
            for (unsigned int i = 0; i < sampleVector.size(); ++i)
            {
                cout << sampleVector.at (i);
                if (i + 1 < sampleVector.size())
                {
                    cout << ", ";
                }
            }
            cout << endl;
        }

        // The data pointer will become invalid after onNewData returns.  When
        // processing the data, it's necessary to either:
        // 1. Do all the processing before this method returns, or
        // 2. Copy the data (not just the pointer) for later processing.
        //
        // The Royale library's depth-processing thread may block while waiting
        // for this function to return; if this function is slow then there
        // may be some lag between capture and onNewData for the next frame.
        // If it's very slow then Royale may drop frames to catch up.
    }
};

int main()
{
    // Windows requires that the application allocate these, not the DLL.
    PlatformResources resources;

    // This is the data listener which will receive callbacks.  It's declared
    // before the cameraDevice so that, if this function exits with a 'return'
    // statement while the camera is still capturing, it will still be in scope
    // until the cameraDevice's destructor implicitly deregisters the listener.
    MyListener listener;

    // this represents the main camera device object
    unique_ptr<ICameraDevice> cameraDevice;

    // the camera manager will query for a connected camera
    {
        CameraManager manager;

        auto camlist = manager.getConnectedCameraList();
        cout << "Detected " << camlist.size() << " camera(s)." << endl;
        if (!camlist.empty())
        {
            cout << "CamID for first device: " << camlist.at (0).c_str() << " with a length of (" << camlist.at (0).length() << ")" << endl;
            cameraDevice = manager.createCamera (camlist[0]);
        }
    }
    // the camera device is now available and CameraManager can be deallocated here

    if (cameraDevice == nullptr)
    {
        cerr << "Cannot create the camera device" << endl;
        return 1;
    }

    // IMPORTANT: call the initialize method before working with the camera device
    if (cameraDevice->initialize() != CameraStatus::SUCCESS)
    {
        cerr << "Cannot initialize the camera device" << endl;
        return 1;
    }

    Vector<String> useCases;
    auto status = cameraDevice->getUseCases (useCases);

    if (status != CameraStatus::SUCCESS || useCases.empty())
    {
        cerr << "No use cases are available" << endl;
        cerr << "getUseCases() returned: " << getErrorString (status) << endl;
        return 1;
    }

    // register a data listener
    if (cameraDevice->registerDataListener (&listener) != CameraStatus::SUCCESS)
    {
        cerr << "Error registering data listener" << endl;
        return 1;
    }

    // choose a use case without mixed mode
    auto selectedUseCaseIdx = 0u;
    auto useCaseFound = false;
    for (auto i = 0u; i < useCases.size(); ++i)
    {
        uint32_t streamCount = 0;
        if (cameraDevice->getNumberOfStreams (useCases[i], streamCount) != CameraStatus::SUCCESS)
        {
            cerr << "Error retrieving the number of streams for use case " << useCases[i] << endl;
            return 1;
        }

        if (streamCount == 1)
        {
            // we found a use case with only one stream
            selectedUseCaseIdx = i;
            useCaseFound = true;
            break;
        }
    }

    // check if we found a suitable use case
    if (!useCaseFound)
    {
        cerr << "Error : There are only mixed modes available" << endl;
        return 1;
    }

    // set an operation mode
    if (cameraDevice->setUseCase (useCases.at (selectedUseCaseIdx)) != CameraStatus::SUCCESS)
    {
        cerr << "Error setting use case" << endl;
        return 1;
    }

    // start capture mode
    if (cameraDevice->startCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error starting the capturing" << endl;
        return 1;
    }

    // let the camera capture for some time
    this_thread::sleep_for (chrono::seconds (5));

    // change the exposure time (limited by the used operation mode [microseconds]
    if (cameraDevice->setExposureTime (200) != CameraStatus::SUCCESS)
    {
        cerr << "Cannot set exposure time" << endl;
    }
    else
    {
        cout << "Changed exposure time to 200 microseconds ..." << endl;
    }

    // let the camera capture for some time
    this_thread::sleep_for (chrono::seconds (5));

    // stop capture mode
    if (cameraDevice->stopCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error stopping the capturing" << endl;
        return 1;
    }

    return 0;
}
