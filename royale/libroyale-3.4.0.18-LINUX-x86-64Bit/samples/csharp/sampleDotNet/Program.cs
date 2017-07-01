using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using RoyaleDotNet;

namespace sampleDotNet
{
    class Program
    {
        class DataReceiver : IDepthDataListener
        {
            public void OnNewData (DepthData data)
            {
                DepthPoint dp = data.points[ (data.height * data.width) / 2];
                Console.WriteLine ("============================================================\n");
                Console.WriteLine ("Received Frame: " + data.width + "x" + data.height + ", some point: { " + dp.x + ", " + dp.y + ", " + dp.z + " } confidence: " + dp.depthConfidence);
            }
        }

        /// <summary>
        /// Read and print the version of Royale that is being used.
        /// </summary>
        static void PrintRoyaleLibraryVersion()
        {
            // There are two methods to get the version number, both will give the same result.
            UInt32 major, minor, patch, build;
            Royale.GetVersion (out major, out minor, out patch, out build);
            Console.WriteLine ("Royale library version: " + major + "." + minor + "." + patch + "." + build);

            // Reading the same information via the RoyaleDotNet Assembly.  Royale's version
            // numbers are major.minor.patch.build, so in System.Version the patch is in
            // version.Build and the build is in version.Revision.
            Version version = typeof (RoyaleDotNet.CameraDevice).Assembly.GetName().Version;
            Console.WriteLine ("RoyaleDotNet version: " + version);

            // It's also possible to read the version control revision that it was built from.
            String scm;
            CameraStatus status = Royale.GetVersion (out major, out minor, out patch, out build, out scm);
            if (CameraStatus.SUCCESS == status)
            {
                // This is not expected to fail, unless something is mismatched with the version numbers,
                // because the only error condition that should make it fail is running out of memory.
                Console.WriteLine ("This library was built from SCM version " + scm);
            }
        }

        static void Main (string[] args)
        {
            Console.WriteLine ("============================================================");
            Console.WriteLine ("|                    royale .NET sample                    |");
            Console.WriteLine ("============================================================");
            PrintRoyaleLibraryVersion();
            Console.WriteLine ("");

            CameraStatus status;
            DataReceiver receiver = new DataReceiver();
            CameraManager camManager = new CameraManager();
            List<string> connectedCameras = camManager.GetConnectedCameraList();

            if (connectedCameras.Count == 0)
            {
                Console.WriteLine ("No connected cameras found.");
                return;
            }

            CameraDevice device = camManager.CreateCamera (connectedCameras[0]);

            status = device.Initialize();
            if (CameraStatus.SUCCESS != status)
            {
                Console.WriteLine ("Failed to initialize camera.");
            }

            status = device.RegisterDepthDataListener (receiver);
            if (CameraStatus.SUCCESS != status)
            {
                Console.WriteLine ("Failed to register data listener.");
            }

            Console.WriteLine ("Starting to capture for 10 seconds.");

            status = device.StartCapture();
            if (CameraStatus.SUCCESS != status)
            {
                Console.WriteLine ("Failed to start capture.");
            }
            Thread.Sleep (10000);

            status = device.StopCapture();
            if (CameraStatus.SUCCESS != status)
            {
                Console.WriteLine ("Failed to stop capture.");
            }

        }
    }
}
