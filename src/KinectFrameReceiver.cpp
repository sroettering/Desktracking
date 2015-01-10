#include "KinectFrameReceiver.h"



const char * KINECT_FRAME_SHARED_MEMORY_NAME = "Local\\KinectFrameStreamMemory";
const char * KINECT_FRAME_READY_EVENT_NAME = "Local\\KinectFrameReadyEvent";
const char * KINECT_FRAME_MEMORY_ACCESS_MUTEX = "Local\\KinectFramMemoryeAccessMutex";



KinectFrameReceiver::KinectFrameReceiver() {
}

int KinectFrameReceiver::receiveFrames() {
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

    sharedMemory = (SharedMemory *)MapViewOfFile(sharedMemoryHandle, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(SharedMemory));
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
