#include <opencv2/opencv.hpp>
#include "serial/serial.h"
#include <iostream>
#include <unistd.h>

class grbl
{
private:
    serial::Serial serialPort;
public:
    std::string state;
    cv::Vec2f loc , minLoc, maxLoc;
    // cv::Vec2f maxMachineLoc = cv::Vec2f(0,0);
    void home();
    // void updateMachineLoc();
    // void updateLoc();
    // void updateAllLocs();
    void connect(std::string port, int Baudrate, int timeout=1000);
    void setTimeout(int timeout);
    void move(cv::Vec2f moveLoc, float speed, bool wait=true);
    void waitMove();
    void close();
    void center();
    void update(bool setbounds=false);
};