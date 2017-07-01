#pragma once

#include <iostream>

#ifdef _WINDOWS
#include <windows.h>
#endif

namespace sample_utils
{
    /**
     * Some platforms (and certain frameworks on those platforms) require the application to call an
     * initialization method before the library can use certain features.
     *
     * The only one currently affecting us is Windows COM, which is needed for the UVC camera
     * support on Windows.
     *
     * Qt will also create these resources, in a Qt app the application does not need to create them
     * (and Qt will fail to start if the application creates them with conflicting settings).
     */
    class PlatformResources
    {
#ifdef _WINDOWS
    public:
        PlatformResources () :
            m_initializedSuccessfully {false}
        {
            auto hr = CoInitializeEx (NULL, COINIT_APARTMENTTHREADED);
            if (FAILED (hr))
            {
                std::cout << "Can not initialize for the COM framework, UVC devices will not work" << std::endl;
            }
            else
            {
                m_initializedSuccessfully = true;
            }
        }

        ~PlatformResources ()
        {
            if (m_initializedSuccessfully)
            {
                CoUninitialize ();
            }
        }

    private:
        bool m_initializedSuccessfully;
#else
    public:
        PlatformResources () = default;
        ~PlatformResources ()
        {
            // non-trivial destructor to avoid an "unused variable platformResources" warning
            (void) this;
        }
#endif
    };
}
