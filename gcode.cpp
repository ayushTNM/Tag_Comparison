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

// void grbl::updateAllLocs()
// {
//     std::vector<std::string> splitted;
//     std::cout << "now" << std::endl;
//     std::vector<std::string> stats = status();
//     loc = cv::Vec2f(std::stof(stats[1]), std::stof(stats[2]));
//     machineLoc = cv::Vec2f(std::stof(stats[5]), std::stof(stats[6]));
// }

// void grbl::updateMachineLoc()
// {
//     std::vector<std::string> splitted;
//     std::cout << "now" << std::endl;
//     std::vector<std::string> stats = split(status()[3],',');
//     machineLoc = cv::Vec2f(std::stof(stats[0]), std::stof(stats[1]));
// }

// void grbl::updateLoc()
// {
//     std::vector<std::string> splitted;
//     std::cout << "now" << std::endl;
//     std::vector<std::string> stats = split(status()[1],',');
//     loc = cv::Vec2f(std::stof(stats[0]), std::stof(stats[1]));
// }


void grbl::home()
{
    // serialPort.readlines();
    serialPort.readlines();
    update();
    //     std::cout << msg[0] << std::endl;
    // exit(0);
    // sleep(1);
    serialPort.write("$H\n");
    while (serialPort.read(2) != "ok")
    {
    }
    serialPort.readlines();
    update(true);
    // std::cout << minLoc << ", " << maxLoc << std::endl;
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

void grbl::center()
{
    serialPort.write("G1 X0 Y0 F 10000\n");
    update();
    update();
    waitMove();
    maxLoc+=loc;
}

void grbl::waitMove()
{

    // serialPort.write("?\n");
    // std::string result = "<Run";
    // serialPort.readlines();
    // update();
    // update();
    while (state != "<Idle")
    {
        // result = state;
        // std::cout << "Here " << state << std::endl;
        update();
    }
    update();
    // update();
    std::cout << "Done" << std::endl;
}

void grbl::move(cv::Vec2f moveLoc, float speed, bool wait)
{
    std::cout << loc << ", " << moveLoc << ", " << minLoc <<", " << maxLoc << std::endl;
    if (loc[0] + moveLoc[0] > maxLoc[0])
    {
        moveLoc[0] = maxLoc[0] - loc[0];
    }
    if (loc[1] + moveLoc[1] > maxLoc[1])
    {
        moveLoc[1] = maxLoc[1] - loc[1];
    }
    if (loc[0] + moveLoc[0] < minLoc[0])
    {
        moveLoc[0] = minLoc[0]-loc[0];
        // moveLoc[0] = -maxLoc[0];
    }
    if (loc[1] + moveLoc[1] < minLoc[1])
    {
        moveLoc[1] = minLoc[1]-loc[1];
    }
    moveLoc *= 10;
    // serialPort.write("?\n");
    // serialPort.readlines();
    // std::cout << serialPort.readline() << std::endl;
    // std::cout << loc << ", " << moveLoc << ", " << minLoc <<", " << maxLoc << std::endl;
    serialPort.write("$J=G21G91X" + std::to_string(moveLoc[0]) + "Y" + std::to_string(moveLoc[1]) + "F" + std::to_string(speed) + "\n");
    // std::cout << serialPort.readline() << std::endl;
    // std::cout << serialPort.readline() << std::endl;
    // std::cout << serialPort.readline() << std::endl;
    // serialPort.readlines();
    update();
    update();
    if (wait)
    {
        waitMove();
    }
}

void grbl::close()
{
    serialPort.close();
}

void grbl::update(bool setbounds)
{
    serialPort.write("?\n");
    std::vector<std::string> lines = serialPort.readlines();
    std::vector<std::string> stats;
    std::vector<std::string> locData;

    for (int i = 0; i < lines.size(); i++)
    {
        if (lines[i][0] == '<')
        {
            // auto pos= stats.begin() + 1;
            std::cout << lines[i] << std::endl;
            stats = split(lines[i], '|');
            if (stats.size() < 3) {
                update();
                return;
            }
            state = stats[0];
            stats[1] = split(stats[1], ':')[1];
            locData = split(stats[1],',');
            loc = cv::Vec2f(std::stof(locData[0])/10,std::stof(locData[1])/10);
            // stats.erase(pos);
            // stats.insert(pos,locData.begin(),locData.end());
            // pos+=4;
            if (setbounds == true && stats.size() > 3 && stats[3][0] == 'W') {
                stats[3] = split(stats[3], ':')[1];
                locData = split(stats[3],',');
                maxLoc = cv::Vec2f(-std::stof(locData[0])/10,-std::stof(locData[1])/10);
                minLoc = loc;
            }
            // stats.erase(pos);
            // stats.insert(pos,locData.begin(),locData.end());
        }
    }

}