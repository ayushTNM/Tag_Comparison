#include <opencv2/opencv.hpp>
#include "serial/serial.h"
#include <iostream>
#include <unistd.h>

class grbl
{
private:
    serial::Serial serialPort;
    cv::Vec2f maxLoc = cv::Vec2f(0,0);
public:
    cv::Vec2f loc = cv::Vec2f(0,0);
    void home();
    cv::Vec2f readMaxLoc();
    void connect(std::string port, int Baudrate, int timeout=1000);
    void setTimeout(int timeout);
    void move(cv::Vec2f moveLoc, float speed, bool wait=true);
    void waitMove();
    void close();
    std::vector<std::string> status();
};