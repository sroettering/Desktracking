#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <iostream>
#include <conio.h>

typedef struct _CameraSpacePoint {
    float X;
    float Y;
    float Z;
} CameraSpacePoint;

const char * KINECT_FRAME_SHARED_MEMORY_NAME = "Local\\KinectFrameStreamMemory";
const char * KINECT_FRAME_READY_EVENT_NAME = "Local\\KinectFrameReadyEvent";
const char * KINECT_FRAME_MEMORY_ACCESS_MUTEX = "Local\\KinectFramMemoryeAccessMutex";

struct SharedMemory {
    unsigned int elementCount;
    CameraSpacePoint elements[640 * 480];
};


int main() {
    HANDLE frameReadyEvent = OpenEvent(EVENT_MODIFY_STATE, FALSE, KINECT_FRAME_READY_EVENT_NAME);
    if(frameReadyEvent == NULL) {
        std::cerr << "Couldn't open frame ready event" << std::endl;
        return EXIT_FAILURE;
    }

    HANDLE accessMutex = OpenMutex(SYNCHRONIZE, FALSE, KINECT_FRAME_MEMORY_ACCESS_MUTEX);
    if(accessMutex == NULL) {
        std::cerr << "Couldn't open memory access mutex" << std::endl;
        return EXIT_FAILURE;
    }

    HANDLE sharedMemoryHandle = OpenFileMapping(FILE_MAP_ALL_ACCESS, false, KINECT_FRAME_SHARED_MEMORY_NAME);
    if(sharedMemoryHandle == NULL) {
        std::cerr << "Couldn't open shared memory" << std::endl;
        return EXIT_FAILURE;
    }

    SharedMemory * sharedMemory = (SharedMemory *)MapViewOfFile(sharedMemoryHandle, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(SharedMemory));
    if(sharedMemory == nullptr) {
        std::cerr << "Couldn't open shared memory" << std::endl;
        return EXIT_FAILURE;
    }

    while(!_kbhit()) {
        WaitForSingleObject(frameReadyEvent, INFINITE);
        WaitForSingleObject(accessMutex, INFINITE);

        std::cout << "received frame" << std::endl;

        ReleaseMutex(accessMutex);

    }

    UnmapViewOfFile(sharedMemory);
    CloseHandle(sharedMemoryHandle);
    CloseHandle(accessMutex);
    CloseHandle(frameReadyEvent);

    return EXIT_SUCCESS;
}
