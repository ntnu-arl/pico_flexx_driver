/****************************************************************************\
 * Copyright (C) 2016 pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#pragma once

namespace royale
{
    /**
     *  This enum defines the access level
     */
    enum class CameraAccessLevel
    {
        L1 = 0, //!< Level 1 access provides depth data using standard, known-working configurations
        L2 = 1, //!< Level 2 access provides raw data, e.g. for custom processing pipelines
        L3 = 2, //!< Level 3 access enables you to overwrite exposure limits
        L4 = 3  //!< Level 4 access is for bringing up new camera modules
    };
}
