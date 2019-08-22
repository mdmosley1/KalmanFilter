#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <cmath> // M_PI, cos, sin
#include <cstdlib> // rand
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <memory> // unique ptr

using namespace cv;

double dt = 0.1; // the time delta for each iteration
const int MAP_SIZE = 500;

Point2f wayPointLocation;

struct Velocity
{
    Velocity(double _lin, double  _ang): linear(_lin), angular(_ang) {};
    double linear, angular;
};

struct Control
{
    Control(double _acc, double  _sa): acceleration(_acc), steerAngle(_sa) {};
    double acceleration, steerAngle;
};


struct State
{
    State(double _x,double _y,double _theta): x(_x), y(_y), theta(_theta) {};
    State(): x(0), y(0), theta(0) {} ;
    double x,y,theta;
};

class Controller
{
public:
    Controller() {};
    virtual Velocity GenerateRobotControl(State _state, Point2f _goal, int key) = 0;

    Velocity vel_ = Velocity(0.0, 0.0);

    const double linearVelMax_ = 100;
    const double linearVelMin_ = 0;

    const double angularVelMax_ = 0.5;
    const double angularVelMin_ = -0.5;
};


class UserController : public Controller
{
public:
    UserController() {};

    // Process User Input
    Control GenerateRobotControl(State _state, Point2f _goal, int key)
        {
            if (key == 'w')
                IncreaseLinearVelocity();
            else if (key == 's')
                DecreaseLinearVelocity();
            else if (key == 'a')
                DecreaseAngularVelocity();
            else if (key == 'd')
                IncreaseAngularVelocity();

            return vel_;
        }

    void IncreaseLinearVelocity()
        {
            double newVel = vel_.linear + linearDelta_;
            vel_.linear = std::min(newVel, linearVelMax_);
        }

    void DecreaseLinearVelocity()
        {
            double newVel = vel_.linear- linearDelta_;
            vel_.linear = std::max(newVel, linearVelMin_);
        }

    void IncreaseAngularVelocity()
        {
            double newAngVel = vel_.angular + angularDelta_;
            vel_.angular = std::min(newAngVel, angularVelMax_);
        }

    void DecreaseAngularVelocity()
        {
            double newAngVel = vel_.angular - angularDelta_;
            vel_.angular = std::max(newAngVel, angularVelMin_);
        }

private:
    const double linearDelta_ = 1;
    const double angularDelta_ = 0.1;
};

// Differential Drive Controller
class DiffDriveController : public Controller
{
public:
    DiffDriveController() {};

    Velocity GenerateRobotControl(State _state, Point2f _goal, int key)
        {
            //double kp = 0.2;
            double kp = 0.1;
            double ka = 0.3;
            double kb = 0;

            double deltaX = _goal.x - _state.x;
            double deltaY = _goal.y - _state.y;

            double rho = std::sqrt(deltaX*deltaX + deltaY*deltaY); // distance btwne goal and state
            double alpha = -_state.theta + std::atan2(deltaY, deltaX); // How much robot needs to turn in order to face waypoint

            if (alpha > M_PI)
                alpha -= 2 * M_PI;
            else if (alpha < -M_PI)
                alpha += 2 * M_PI;

            double beta = -_state.theta - alpha; // desired theta when robot reaches goal (I think)
            double angularVelocity = ka * alpha + kb * beta;
            double linearVelocity = kp * rho - std::fabs(alpha)*20.0; // if alpha is high, then need to reduce the linear velocity

            if (rho < 20.0)
                wayPointLocation= Point2f(rand() % MAP_SIZE,rand() % MAP_SIZE);

            double minVelocity = -5.0;
            double maxVelocity = 10.0;
            linearVelocity = min(max(linearVelocity, minVelocity), maxVelocity);

            return Velocity(linearVelocity, angularVelocity);
        }    
};



void DrawWaypoints(Mat image)
{
    Scalar color = Scalar(255,0,0); // red
    circle(image, wayPointLocation, 5, color, CV_FILLED);
}

void DrawRotatedRectangle(cv::Mat& image, cv::Point centerPoint,
                          cv::Size rectangleSize, double rotationDegrees)
{
    cv::Scalar color = cv::Scalar(0, 255.0, 0); // green

    // Create the rotated rectangle
    cv::RotatedRect rotatedRectangle(centerPoint, rectangleSize, rotationDegrees);

    // We take the edges that OpenCV calculated for us
    cv::Point2f vertices2f[4];
    rotatedRectangle.points(vertices2f);

    // Convert them so we can use them in a fillConvexPoly
    cv::Point vertices[4];
    for(int i = 0; i < 4; ++i)
        vertices[i] = vertices2f[i];

    // Now we can fill the rotated rectangle with our specified color
    cv::fillConvexPoly(image,
                       vertices,
                       4,
                       color);
}

class Robot
{
public:
    Robot(double _x, double _y) : pos_(_x, _y)
        {
        }

    void DrawFrontBumper(Mat& image)
        {
            Point2f frontPosition = pos_ + 9*Point2f(cos(theta_),sin(theta_));
            circle(image, frontPosition, 5, color_, CV_FILLED);
        }

    void UpdateVelocity(Velocity vel)
        {
            linearVel_ = vel.linear;
            angularVel_ = vel.angular;
        }

    void Draw(Mat& image )
        {
            // Draw the main body of the robot
            //Point2f tl = pos_ - Point2f( cos(theta_)*width_/2, sin(theta_)*height_/2);
            //Point2f br = pos_ + Point2f( cos(theta_)*width_/2, sin(theta_)*height_/2);
            DrawRotatedRectangle(image, pos_, Size(width_,height_), (theta_+M_PI/2)*180/M_PI);
            DrawFrontBumper(image);
            //rectangle(image, tl, br, Scalar(0,255,0), CV_FILLED);

            // Draw wheel top left
            // DrawWheelTL(image);
            // DrawWheelTR(image);
            // DrawWheelBL(image);
            // DrawWheelBR(image);
        }

    void UpdatePosition()
        {
            pos_.y +=  dt*linearVel_*sin(theta_);
            pos_.x +=  dt*linearVel_*cos(theta_);
            double steeringRadius = 5;
            angularVel_ = steerAngle*linearVel_/ steeringRadius;
            theta_ += dt*angularVel_*linearVel_/10.0;
        }
    
    
    void DrawWheelTL(Mat& image)
        {
            Point2f p1 = pos_ - Point2f(width_/2,height_/2) - Point2f(wheelWidth_,0);
            Point2f p2 = p1 + Point2f(wheelWidth_, wheelHeight_);
            rectangle(image, p1, p2, Scalar(0,255,0), CV_FILLED);
        }

    void DrawWheelTR(Mat& image)
        {
            Point2f p1 = pos_ + Point2f(width_/2, -height_/2);
            Point2f p2 = p1 + Point2f(wheelWidth_, wheelHeight_);
            rectangle(image, p1, p2, Scalar(0,255,0), CV_FILLED);
        }
    void DrawWheelBL(Mat& image)
        {
            Point2f p2 = pos_ + Point2f(-width_/2, height_/2);
            Point2f p1 = p2 - Point2f(wheelWidth_, wheelHeight_);
            rectangle(image, p1, p2, Scalar(0,255,0), CV_FILLED);
        }
    void DrawWheelBR(Mat& image)
        {
            Point2f p1 = pos_ + Point2f(width_/2, height_/2) - Point2f(0, wheelHeight_);
            Point2f p2 = p1 + Point2f(wheelWidth_, wheelHeight_);
            rectangle(image, p1, p2, Scalar(0,255,0), CV_FILLED);
        }

    Point2f pos_;
    double theta_ = 0;
private:
    double width_ = 10;
    double height_ = 20;

    double linearVel_ = 10; // m/s
    double angularVel_ = 0; // rads/s

        

    int wheelWidth_ = 2;
    int wheelHeight_ = 4;

    const Scalar color_ = Scalar(0, 255, 0); // green
};

void ClearScreen(Mat& image)
{
    image = Mat::zeros( MAP_SIZE, MAP_SIZE, CV_8UC3 );
}

class KalmanFilter1
{
public:
    KalmanFilter1() {};
    virtual void EstimateState() = 0;
};

class KalmanEKF : public KalmanFilter1
{
public:
    KalmanEKF() {};
    void EstimateState()
        {
            std::cout << "EKF filter." << "\n";
        }
};

class KalmanLinear : public KalmanFilter1
{
public:
    KalmanLinear() {};
    void EstimateState()
        {
            std::cout << "Linear kalman filter." << "\n";
        }
};

int main(int argc, char** argv)
{
    //initialize a 120X350 matrix of black pixels:
    Mat output;
    ClearScreen(output);

    /* initialize random seed: */
    srand (time(NULL));
    wayPointLocation= Point2f(rand() % MAP_SIZE,rand() % MAP_SIZE);

    Robot robot(100,200); // start a new robot at this position

    std::cout << "Press q to quit." << "\n";

    std::unique_ptr<KalmanFilter1> kf;
    std::unique_ptr<Controller> controller;

    bool autonomousMode = false;
    if (autonomousMode)
        controller = std::make_unique<DiffDriveController>();
    else
        controller = std::make_unique<UserController>();

    //kf = std::make_unique<KalmanLinear>();
    //kf = std::make_unique<KalmanEKF>();
    int key = 0;
    while(key != 'q')
    {
        // Process user input
        key = waitKey(1.0);
        //kf->EstimateState();
        State state;
        state.x = robot.pos_.x;
        state.y = robot.pos_.y;
        state.theta = robot.theta_;
        //Velocity vel = controller->GenerateRobotControl(state, wayPointLocation, key);
        Control control = controller->GenerateRobotControl(state, wayPointLocation, key);
        robot.UpdateVelocity(control);
        // update the positions of everything
        robot.UpdatePosition();
        // Clear the screen before drawing
        ClearScreen(output);
        // draw everything to the output matrix
        robot.Draw(output);
        DrawWaypoints(output);
        // display the matrix
        imshow("Output", output);
    }
    
    return 0;
}
