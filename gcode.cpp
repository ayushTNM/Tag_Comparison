#include "gcode.h"

std::vector<std::string> split(std::string s, char del)
{
    std::stringstream ss(s);
    std::string word;
    std::vector<std::string> splitted;
    while (!ss.eof())
    {
        getline(ss, word, del);
        splitted.push_back(word);
    }
    return splitted;
}

cv::Vec2f grbl::readMaxLoc()
{
    std::vector<std::string> splitted;
    std::cout << "now" << std::endl;
    std::vector<std::string> stats = status();
    return (cv::Vec2f(std::stof(stats[1]), std::stof(stats[2])));
}

void grbl::home()
{
    serialPort.read();
    sleep(1);
    serialPort.write("$H\n");
    while (serialPort.read(2) != "ok")
    {
    }
    serialPort.readlines();
    maxLoc = readMaxLoc();
    maxLoc[0] = (-maxLoc[0]);
    maxLoc[1] = (-maxLoc[1]);
    std::cout << maxLoc << std::endl;
}

void grbl::connect(std::string port, int baudrate, int timeout)
{
    serialPort.setPort(port);
    serialPort.setBaudrate(baudrate);
    serialPort.setTimeout(serial::Timeout::max(), timeout, 0, timeout, 0);
    serialPort.open();
}

void grbl::setTimeout(int timeout)
{
    serialPort.setTimeout(serial::Timeout::max(), timeout, 0, timeout, 0);
}

void grbl::waitMove()
{

    serialPort.write("?\n");
    std::string result = "<Run";
    serialPort.readlines();
    while (result == "<Run")
    {
        result = status()[0];
    }
}

void grbl::move(cv::Vec2f moveLoc, float speed, bool wait)
{
    moveLoc *= 10;
    std::cout << loc << ", " << moveLoc << ", " << maxLoc << std::endl;
    if (loc[0] + moveLoc[0] > maxLoc[0])
    {
        moveLoc[0] = maxLoc[0] - loc[0];
    }
    if (loc[1] + moveLoc[1] > maxLoc[1])
    {
        moveLoc[1] = maxLoc[1] - loc[1];
    }
    if (loc[0] + moveLoc[0] < 0)
    {
        moveLoc[0] = loc[0];
        // moveLoc[0] = -maxLoc[0];
    }
    if (loc[1] + moveLoc[1] < 0)
    {
        moveLoc[1] = loc[1];
    }
    serialPort.write("?\n");
    std::cout << serialPort.readline() << std::endl;
    std::cout << loc << ", " << moveLoc << ", " << maxLoc << std::endl;
    serialPort.write("G21G91G1X" + std::to_string(moveLoc[0]) + "Y" + std::to_string(moveLoc[1]) + "F" + std::to_string(speed) + "\nG90G21\n");
    std::cout << serialPort.readline() << std::endl;
    if (wait)
    {
        waitMove();
        loc += moveLoc;
    }
    else
        loc += moveLoc;
}

void grbl::close()
{
    serialPort.close();
}

std::vector<std::string> grbl::status()
{
    serialPort.write("?\n");
    std::vector<std::string> lines = serialPort.readlines();
    std::vector<std::string> splitted = std::vector<std::string>();

    for (int i = 0; i < lines.size(); i++)
    {
        if (lines[i][0] == '<')
        {
            splitted = split(lines[i], ',');
            splitted[1] = split(splitted[1], ':')[1];
            return splitted;
        }
    }
    return splitted;
}