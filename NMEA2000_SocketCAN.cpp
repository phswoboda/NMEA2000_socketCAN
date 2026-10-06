
/*
NMEA2000_SocketCAN.cpp

2017 Copyright (c) Al Thomason   All rights reserved

Support the socketCAN access (ala, Linux, RPi)
See: https://github.com/thomasonw/NMEA2000_socketCAN
     https://github.com/ttlappalainen/NMEA2000


Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.


THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Inherited NMEA2000 object for socketCAN
See also NMEA2000 library.
*/


#include "NMEA2000_SocketCAN.h"

#include <stdio.h>
#include <iostream>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cerrno>

//*****************************************************************************
//  Pass in pointer to character array which contains (or will contain) the
//  string of the CANsocket to use in :open().   If no paramater is passed in,
//  or NULL is passed in, the defalt socket 'can0' will be used
tNMEA2000_SocketCAN::tNMEA2000_SocketCAN(char* CANport) : tNMEA2000()
{
    static char defaultCANport[] = "can0";

    if (CANport != NULL)
        _CANport = CANport;
    else
        _CANport = defaultCANport;                                                 // NULL passed in, set to port to default: CAN0

}



//*****************************************************************************
void tNMEA2000_SocketCAN::SetCANport(char *CANport) {
    if (CANport != NULL)
       _CANport = CANport;
}



//*****************************************************************************
tNMEA2000_SocketCAN::~tNMEA2000_SocketCAN() {
    if (skt >= 0) close(skt);
}

void tNMEA2000_SocketCAN::CloseSocket() {
    if (skt >= 0) close(skt);
    skt = -1;
    OpenState = os_OpenCAN;
    OpenScheduler.FromNow(1000);
}

bool tNMEA2000_SocketCAN::FailOpen(const char *message) {
    const std::string error = std::string(message) + ": " + _CANport;
    if (mLastError != error) cerr << error << endl;
    mLastError = error;
    CloseSocket();
    return false;
}

bool tNMEA2000_SocketCAN::CANOpen() {
    if (skt >= 0) close(skt);
    skt = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    if (skt < 0) return FailOpen("Failed open CAN socket");
    struct ifreq ifr = {};
    strncpy(ifr.ifr_name, _CANport.c_str(), sizeof(ifr.ifr_name) - 1);
    if (ioctl(skt, SIOCGIFINDEX, &ifr) < 0) return FailOpen("Failed CAN ioctl");
    mIfIndex = ifr.ifr_ifindex;
    if (ioctl(skt, SIOCGIFFLAGS, &ifr) < 0 || !(ifr.ifr_flags & IFF_UP))
        return FailOpen("CAN interface down");
    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = mIfIndex;
    if (bind(skt, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
        return FailOpen("Failed CAN bind");
    mLastError.clear();
    mNextLinkCheck = millis() + 1000;
    return true;
}


//*****************************************************************************
bool tNMEA2000_SocketCAN::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent) {
    if (skt < 0 || len > 8) return false;
    struct can_frame frame = {};
    frame.can_id = id | CAN_EFF_FLAG;
    frame.can_dlc = len;
    memcpy(frame.data, buf, len);
    const auto sent = write(skt, &frame, sizeof(frame));
    if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != ENOBUFS)
        CloseSocket();
    return sent == sizeof(frame);
}

bool tNMEA2000_SocketCAN::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) {
    if (skt < 0) return false;
    if (int32_t(millis() - mNextLinkCheck) >= 0) {
        struct ifreq ifr = {};
        strncpy(ifr.ifr_name, _CANport.c_str(), sizeof(ifr.ifr_name) - 1);
        if (ioctl(skt, SIOCGIFINDEX, &ifr) < 0 || ifr.ifr_ifindex != mIfIndex
            || ioctl(skt, SIOCGIFFLAGS, &ifr) < 0 || !(ifr.ifr_flags & IFF_UP)) {
            CloseSocket();
            return false;
        }
        mNextLinkCheck = millis() + 1000;
    }
    struct can_frame frame = {};
    const auto received = read(skt, &frame, sizeof(frame));
    if (received != sizeof(frame)) {
        if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
            CloseSocket();
        return false;
    }
    if (!(frame.can_id & CAN_EFF_FLAG) || frame.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG) || frame.can_dlc > 8)
        return false;
    memcpy(buf, frame.data, frame.can_dlc);
    len = frame.can_dlc;
    id = frame.can_id & CAN_EFF_MASK;
    return true;
}


//*****************************************************************************
tSocketStream::tSocketStream(const char *_port) : port(-1) {
  if ( _port!=0 ) {
    port=open(_port, O_RDWR | O_NOCTTY | O_NDELAY);
  }

  if ( port!=-1 ) {
    cout << _port << " opened" << endl;
  } else {
    if ( _port!=0 ) cerr << "Failed to open port " << _port << ". Using stdout"<< endl;
  }
}

//*****************************************************************************
tSocketStream::~tSocketStream() {
  if ( port!=-1 ) {
    close(port);
  }
}


/********************************************************************
*	Other 'Bridge' functions and classes
*
*
*
**********************************************************************/
int tSocketStream::read() {
  if ( port!=-1 ) {
    return -1;
  } else {
    // Serial stream bridge -- Returns first byte if incoming data, or -1 on no available data.
    struct timeval tv = { 0L, 0L };
    fd_set fds;

    FD_ZERO(&fds);
    FD_SET(0, &fds);
    if (select(1, &fds, NULL, NULL, &tv) < 0)                                   // Check fd=0 (stdin) to see if anything is there (timeout=0)
        return -1;                                                              // Nothing is waiting for us.

   return (getc(stdin));                                                         // Something is there, go get one char of it.
  }
}



//*****************************************************************************
size_t tSocketStream::write(const uint8_t* data, size_t size) {                // Serial Stream bridge -- Write data to stream.
  if ( port!=-1 ) {
    return ::write(port,data,size);
  } else {
    size_t i;

    for (i=0; (i<size) && data[i];  i++)                                        // send chars to stdout for 'size' or until null is found.
        putc(data[i],stdout);

    return(i);
  }
}


// std::this_thread::sleep_for(std::chrono::milliseconds(x));
// http://stackoverflow.com/questions/4184468/sleep-for-milliseconds

//*****************************************************************************
void delay(const uint32_t ms) {
    usleep(ms*1000);
};


//*****************************************************************************
uint32_t millis(void) {
    struct timespec ticker;

    clock_gettime(CLOCK_MONOTONIC, &ticker);
    return ((uint32_t) ((ticker.tv_sec * 1000) + (ticker.tv_nsec / 1000000)));

};




