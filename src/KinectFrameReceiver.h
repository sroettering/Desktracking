#ifndef KINECTFRAMERECEIVER_H
#define KINECTFRAMERECEIVER_H

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <iostream>
#include <conio.h>

typedef struct _CameraSpacePoint {
    float X;
    float Y;
    float Z;
} CameraSpacePoint;

struct SharedMemory {
    unsigned int elementCount;
    CameraSpacePoint elements[640 * 480];
};

class KinectFrameReceiver {
public:
    KinectFrameReceiver();
    int receiveFrames();

    SharedMemory *sharedMemory;
private:
};

#endif // KINECTFRAMERECEIVER_H
