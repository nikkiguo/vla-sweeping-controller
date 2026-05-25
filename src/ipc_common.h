#pragma once
#include <atomic>

#pragma pack(push, 1)

// Data container for shared memory between the simulation and VLA/external processes
struct SharedDataStruct {
    std::atomic<uint64_t> frame_index; // Prevent tearing
    double joint_states[6];
    uint8_t camera_pixels[640 * 480 * 3];
};
#pragma pack(pop)