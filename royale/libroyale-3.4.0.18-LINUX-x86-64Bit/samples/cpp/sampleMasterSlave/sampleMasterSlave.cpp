#include <royale.hpp>
#include <iostream>
#include <thread>
#include <chrono>

#include <sample_utils/PlatformResources.hpp>

using namespace royale;
using namespace sample_utils;
using namespace std;

class MasterListener : public IDepthDataListener
{
    void onNewData (const DepthData *data)
    {
        cout << "Received data for the master ..." << endl;

        // Do something with the data
    }
};

class SlaveListener : public IDepthDataListener
{
    void onNewData (const DepthData *data)
    {
        cout << "Received data for the slave ..." << endl;

        // Do something with the data
    }
};

// This example shows how to handle master/slave connections.
// It tries to open two cameras and define the first one
// as master and the second one as slave. For this you should
// use two modules of the same kind.
int main()
{
    // Windows requires that the application allocate these, not the DLL.
    PlatformResources resources;

    // These are the data listeners which will receive callbacks.
    SlaveListener slaveListener;
    MasterListener masterListener;

    // these represent the main camera device objects
    unique_ptr<ICameraDevice> slaveCameraDevice;
    unique_ptr<ICameraDevice> masterCameraDevice;

    // the camera manager will query for a connected camera
    {
        CameraManager manager;

        auto camlist = manager.getConnectedCameraList();
        if (camlist.size() < 2)
        {
            cerr << "The master/slave example needs at least 2 cameras!" << endl;
            return 1;
        }
        cout << "Detected " << camlist.size() << " camera(s)." << endl;

        // For this simplified example we assume that the first found camera will be the
        // master and the second one the slave. At this point you might want to open
        // specific devices based on their ID!

        cout << "CamID for master : " << camlist.at (0).c_str() << endl;
        masterCameraDevice = manager.createCamera (camlist[0], TriggerMode::MASTER);

        cout << "CamID for slave : " << camlist.at (1).c_str() << endl;
        slaveCameraDevice = manager.createCamera (camlist[1], TriggerMode::SLAVE);
    }

    // the camera devices should now be available and the CameraManager can be deallocated here

    if (masterCameraDevice == nullptr ||
            slaveCameraDevice == nullptr)
    {
        cerr << "Cannot create the camera devices" << endl;
        return 1;
    }

    // IMPORTANT: call the initialize method of the master before initializing the slave!
    if (masterCameraDevice->initialize() != CameraStatus::SUCCESS)
    {
        cerr << "Cannot initialize the master" << endl;
        return 1;
    }
    if (slaveCameraDevice->initialize() != CameraStatus::SUCCESS)
    {
        cerr << "Cannot initialize the slave" << endl;
        return 1;
    }

    // register the data listeners
    if (masterCameraDevice->registerDataListener (&masterListener) != CameraStatus::SUCCESS)
    {
        cerr << "Error registering data listener for the master" << endl;
        return 1;
    }
    if (slaveCameraDevice->registerDataListener (&slaveListener) != CameraStatus::SUCCESS)
    {
        cerr << "Error registering data listener for the slave" << endl;
        return 1;
    }

    // retrieve available use cases
    Vector<String> useCasesMaster;
    if (masterCameraDevice->getUseCases (useCasesMaster) != CameraStatus::SUCCESS)
    {
        cerr << "Error retrieving use cases for the master" << endl;
        return 1;
    }

    Vector<String> useCasesSlave;
    if (slaveCameraDevice->getUseCases (useCasesSlave) != CameraStatus::SUCCESS)
    {
        cerr << "Error retrieving use cases for the slave" << endl;
        return 1;
    }

    // choose a use case without mixed mode
    auto selectedUseCaseIdx = 0u;
    auto useCaseFound = false;
    for (auto i = 0u; i < useCasesMaster.size(); ++i)
    {
        uint32_t streamCount = 0;
        if (masterCameraDevice->getNumberOfStreams (useCasesMaster[i], streamCount) != CameraStatus::SUCCESS)
        {
            cerr << "Error retrieving the number of streams for use case " << useCasesMaster[i] << endl;
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

    // We expect both devices to be the same kind, and therefore have the same list of
    // available use cases. Use the name of the master's use case for the slave, if it isn't
    // an available use case on the slave then something is wrong.
    //
    // The use case name includes the exposure time, so this prevents having a master
    // driving a slave faster than the slave's use case is expected to run.
    const auto masterUseCase = useCasesMaster.at (selectedUseCaseIdx);

    // set an appropriate use case (in this case we take the first one)
    if (masterCameraDevice->setUseCase (masterUseCase) != CameraStatus::SUCCESS)
    {
        cerr << "Error setting use case '" << masterUseCase << "' for the master" << endl;
        return 1;
    }
    if (slaveCameraDevice->setUseCase (masterUseCase) != CameraStatus::SUCCESS)
    {
        cerr << "Error setting use case '" << masterUseCase << "' for the slave" << endl;
        return 1;
    }

    // IMPORTANT: at this point it is important to first start capturing for all the slave cameras,
    // as they will be triggered by the master!
    if (slaveCameraDevice->startCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error starting the capturing for the slave" << endl;
        return 1;
    }
    if (masterCameraDevice->startCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error starting the capturing for the master" << endl;
        return 1;
    }


    // let the cameras capture for some time
    this_thread::sleep_for (chrono::seconds (5));


    // IMPORTANT: stop capturing for the master before stopping the capturing for the slaves, otherwise
    // the slave might be triggered after the slaveCameraDevice is already stopped!
    if (masterCameraDevice->stopCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error stopping the capturing for the master" << endl;
        return 1;
    }
    if (slaveCameraDevice->stopCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error stopping the capturing for the slave" << endl;
        return 1;
    }

    return 0;
}
