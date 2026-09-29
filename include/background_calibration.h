#pragma once

#include <stddef.h>


namespace BackgroundCalibration
{
    void reset();

    bool addCurrentFrame();

    bool finalize();

    bool ready();

    size_t capturedFrames();

    size_t requiredFrames();

    void analyzeCurrentFrame();
}