#ifndef KINECTFRAMERECEIVER_H
#define KINECTFRAMERECEIVER_H

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <iostream>
#include <conio.h>

class KinectFrameReceiver {
public:
    KinectFrameReceiver();
    int receiveFrames();
private:
};

#endif // KINECTFRAMERECEIVER_H
