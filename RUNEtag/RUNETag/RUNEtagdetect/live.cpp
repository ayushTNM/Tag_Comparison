#include <opencv2/opencv.hpp>

using namespace cv;

using namespace std;

int main() {

Mat image;

namedWindow("Display window");

VideoCapture cap(0);

if (!cap.isOpened()) {

cout << "cannot open camera";

}

while (true) {

cap >> image;

imshow("Display window", image);

if(waitKey(30) == 27) // Wait for 'esc' key press to exit
        { 
            break; 
        }

}

return 0;

}
