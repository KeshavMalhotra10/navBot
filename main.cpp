#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;

// START IQ MACROS
#define waitUntil(condition) \
    do                       \
    {                        \
        wait(5, msec);       \
    } while (!(condition))

#define repeat(iterations) \
    for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS

// Robot configuration code.
inertial BrainInertial = inertial();
motor MotorLeft = motor(PORT12, false);
motor MotorRight = motor(PORT7, true);
optical OpticalSensor = optical(PORT9);
distance DistanceSensor = distance(PORT8);
bumper BumperSensor = bumper(PORT5);
touchled TouchSensor = touchled(PORT10);
controller Controller = controller();
motor MotorBox = motor(PORT1, true);
motor MotorLock = motor(PORT6, true);

// generating and setting random seed
void initializeRandomSeed()
{
    wait(100, msec);
    double xAxis = BrainInertial.acceleration(xaxis) * 1000;
    double yAxis = BrainInertial.acceleration(yaxis) * 1000;
    double zAxis = BrainInertial.acceleration(zaxis) * 1000;
    // Combine these values into a single integer
    int seed = int(
        xAxis + yAxis + zAxis);
    // Set the seed
    srand(seed);
}

// Converts a color to a string
const char *convertColorToString(color col)
{
    if (col == colorType::red)
        return "red";
    else if (col == colorType::green)
        return "green";
    else if (col == colorType::blue)
        return "blue";
    else if (col == colorType::white)
        return "white";
    else if (col == colorType::yellow)
        return "yellow";
    else if (col == colorType::orange)
        return "orange";
    else if (col == colorType::purple)
        return "purple";
    else if (col == colorType::cyan)
        return "cyan";
    else if (col == colorType::black)
        return "black";
    else if (col == colorType::transparent)
        return "transparent";
    else if (col == colorType::red_violet)
        return "red_violet";
    else if (col == colorType::violet)
        return "violet";
    else if (col == colorType::blue_violet)
        return "blue_violet";
    else if (col == colorType::blue_green)
        return "blue_green";
    else if (col == colorType::yellow_green)
        return "yellow_green";
    else if (col == colorType::yellow_orange)
        return "yellow_orange";
    else if (col == colorType::red_orange)
        return "red_orange";
    else if (col == colorType::none)
        return "none";
    else
        return "unknown";
}

// Convert colorType to string
const char *convertColorToString(colorType col)
{
    if (col == colorType::red)
        return "red";
    else if (col == colorType::green)
        return "green";
    else if (col == colorType::blue)
        return "blue";
    else if (col == colorType::white)
        return "white";
    else if (col == colorType::yellow)
        return "yellow";
    else if (col == colorType::orange)
        return "orange";
    else if (col == colorType::purple)
        return "purple";
    else if (col == colorType::cyan)
        return "cyan";
    else if (col == colorType::black)
        return "black";
    else if (col == colorType::transparent)
        return "transparent";
    else if (col == colorType::red_violet)
        return "red_violet";
    else if (col == colorType::violet)
        return "violet";
    else if (col == colorType::blue_violet)
        return "blue_violet";
    else if (col == colorType::blue_green)
        return "blue_green";
    else if (col == colorType::yellow_green)
        return "yellow_green";
    else if (col == colorType::yellow_orange)
        return "yellow_orange";
    else if (col == colorType::red_orange)
        return "red_orange";
    else if (col == colorType::none)
        return "none";
    else
        return "unknown";
}

void vexcodeInit()
{

    // Initializing random seed.
    initializeRandomSeed();
}

// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

//----------------------------------------------------------------------------
//
//    Module:       main.cpp
//    Author:       {author}
//    Created:      {date}
//    Description:  IQ project
//
//----------------------------------------------------------------------------

// Include the IQ Library
#include "iq_cpp.h"
// safety feature if next move is a turn it should make a noise
#include <string>
#include <vector>
#include <limits> //allows for a proper shortest path implementation
// Allows for easier use of the VEX Library
using namespace vex;
// void configureAllSensors_DONT_USE(){ //will be left for now... need to be changed
//   //DO NOT CALL THIS FUNCTION
//   BrainInertial.calibrate();
//   wait(2,seconds);
//   BrainInertial.setHeading(0,degrees);
//   BrainInertial.setRotation(0,degrees);
//   MotorLeft.setPosition(0,turns);
//   MotorRight.setPosition(0,turns);
//   Brain.Screen.clearScreen();
//   Brain.Screen.setFont(mono15);
//   Optical3.setLight(ledState::on);
// }

// global var and constant
const int MAX_ROOMS = 5; // will make this bigger later
const int DRIVE_UPDATE_INTERVAL_MSEC = 10;
const int WHEEL_CIRCUMFERENCE_MM = 200;
const int DRIVE_MODE = 0;
const int TURN_MODE = 1;
motor_group DriveMotors(MotorLeft, MotorRight);

class Path
{
public:
    static const int MAX_STEPS = 10;
    int startPoint = -1;
    int endPoint = -1;
    double steps[MAX_STEPS][2] = {0};
    int length = 0;
    int totalDistance_mm = 0;
};

void pressToOpen(int waitTime)
{
    double initialTime = Brain.Timer.value();

    while (!TouchSensor.pressing() && (Brain.Timer.value() - initialTime) < waitTime)
    {
    }

    if (!TouchSensor.pressing())
    {
        while (!TouchSensor.pressing())
        {
            Brain.Screen.clearScreen();
            Brain.Screen.print("Please press the button!");
            Brain.playSound(siren);
            wait(1, seconds);
        }
    }

    while (TouchSensor.pressing())
    {
    }
}

void pressToClose()
{
    while (!TouchSensor.pressing())
    {
    }
    while (TouchSensor.pressing())
    {
    }
}

void openBox(double openPosition, double lockPosition, int waitTime)
{

    const double Turns_to_Cm = 20.0;
    pressToOpen(waitTime);
    MotorLock.spin(forward, 7, percent);
    while (MotorLock.position(turns) * Turns_to_Cm < lockPosition)
    {
        wait(20, msec);
    }
    MotorLock.stop(hold);
    MotorBox.spin(forward, 7, percent);
    while (MotorBox.position(turns) * Turns_to_Cm < openPosition)
    {
        wait(20, msec);
    }

    MotorBox.stop(hold);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("audwaidad");
    pressToClose();
    MotorBox.spin(reverse, 7, percent);
    while (MotorBox.position(turns) > 0)
    {
    }
    MotorBox.stop(hold);
    MotorLock.spin(reverse, 7, percent);
    while (MotorLock.position(turns) > 0)
    {
    }
    MotorLock.stop(hold);
}

// ui

// input menu
int getNumberInput(const std::string prompt)
{
    int currentSelection = 1;
    int lastSelection = 1;
    Brain.Screen.print("%d", currentSelection);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.clearScreen();
    Brain.Screen.print("%s", prompt.c_str());
    Brain.Screen.newLine();
    Brain.Screen.print("R-Up: Confirm");
    Brain.Screen.newLine();
    Brain.Screen.print("L-Up: Cancel");
    Brain.Screen.newLine();
    Brain.Screen.print("%d", currentSelection);

    while (Controller.ButtonRUp.pressing() == 0 && Controller.ButtonLUp.pressing() == 0)
    {
        if (currentSelection != lastSelection)
        {
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.clearScreen();
            Brain.Screen.print("%s", prompt.c_str());
            Brain.Screen.newLine();
            Brain.Screen.print("R-Up: Confirm");
            Brain.Screen.newLine();
            Brain.Screen.print("L-Up: Cancel");
            Brain.Screen.newLine();
            Brain.Screen.print("%d", currentSelection);
            lastSelection = currentSelection;
        }
        if (Controller.ButtonEUp.pressing() == 1)
        {
            currentSelection++;

            if (currentSelection > MAX_ROOMS)
            { // wrap around
                currentSelection = 1;
            }

            while (Controller.ButtonEUp.pressing() == 1)
            {
            }
        }
        else if (Controller.ButtonEDown.pressing() == 1)
        {
            currentSelection--;

            if (currentSelection < 1)
            {
                currentSelection = MAX_ROOMS; // wrap around
            }

            while (Controller.ButtonEDown.pressing() == 1)
            {
            }
        }
        wait(5, msec);
    }

    if (Controller.ButtonLUp.pressing())
    {
        currentSelection = -1;
    }

    while (Controller.ButtonRUp.pressing() || Controller.ButtonLUp.pressing())
    {
    } // dont let them potentially go to another menu screen and
    // write a value there accidently by holding this button

    return currentSelection;
}

// stores if you can go from room a to room b by checking the distance value stored in
// adhacencyMatrix[i][j]
void createAdjacencyMatrix(const std::vector<Path> &pathLibrary, double (&adjacencyMatrix)[MAX_ROOMS + 1][MAX_ROOMS + 1])
{
    // resets the array
    for (int i = 0; i <= MAX_ROOMS; i++)
    {
        for (int j = 0; j <= MAX_ROOMS; j++)
        {
            adjacencyMatrix[i][j] = -1;
        }
    }
    // adds each path
    for (int i = 0; i < pathLibrary.size(); i++)
    {
        int start = pathLibrary[i].startPoint;
        int end = pathLibrary[i].endPoint;
        int dist = pathLibrary[i].totalDistance_mm;
        adjacencyMatrix[start][end] = dist;
    }
}
std::vector<int> shortestPath(int start, int end, const double (&adjacencyMatrix)[MAX_ROOMS + 1][MAX_ROOMS + 1])
{
    // basically uses djikstras
    const double INF = std::numeric_limits<double>::infinity();
    double distances[MAX_ROOMS + 1];
    int came_from[MAX_ROOMS + 1];
    bool visited[MAX_ROOMS + 1];

    for (int i = 1; i <= MAX_ROOMS; i++)
    {
        // check the bounds on this later...
        // im not adding a 0 bro thats too hard icl
        // depends if we include 0 as a possible room option
        distances[i] = INF;
        came_from[i] = 0;
        visited[i] = false;
    }

    distances[start] = 0;
    for (int i = 1; i <= MAX_ROOMS; i++)
    {
        double minDist = INF;
        int closestPoint = -1; //-1 indicates not found yet

        // this part finds the node with the least distance that hasnt been visited
        // basically a greedy algo
        for (int j = 1; j <= MAX_ROOMS; j++)
        {
            if (!visited[j] && distances[j] < minDist)
            {
                minDist = distances[j];
                closestPoint = j;
            }
        }

        if (closestPoint != -1)
        {
            visited[closestPoint] = true;

            for (int neighbour = 1; neighbour <= MAX_ROOMS; neighbour++)
            {
                double distNeighbour = adjacencyMatrix[closestPoint][neighbour];

                if (distNeighbour != -1 && !visited[neighbour])
                {
                    double potentialPathDist = distances[closestPoint] + distNeighbour;
                    if (potentialPathDist < distances[neighbour])
                    {
                        distances[neighbour] = potentialPathDist;
                        came_from[neighbour] = closestPoint;
                    }
                }
            }
        }
    }

    // get the literal path now if empty it means there was no path

    std::vector<int> path;

    if (distances[end] < INF)
    {
        int currentPosition = end;

        while (currentPosition != 0)
        { // came_from[start]= 0 is unique unless start and end are same... dont allow that i guess

            path.push_back(currentPosition);
            currentPosition = came_from[currentPosition];
        }

        // so far we have the path in reverse order

        for (int i = 0; i < path.size() / 2; i++)
        {
            int j = path.size() - 1 - i;
            std::swap(path[i], path[j]);
        }
        // now we have path in forward order
    }
    return path;
}

// helper functions

double turns_to_mm(double numTurns)
{
    return numTurns * WHEEL_CIRCUMFERENCE_MM;
}

double mm_to_turns(double distance_mm)
{
    return distance_mm / WHEEL_CIRCUMFERENCE_MM;
}

int findPath(int startNode, int endNode, const std::vector<Path> &pathLibrary)
{
    for (int i = 0; i < pathLibrary.size(); ++i)
    {
        if (pathLibrary[i].startPoint == startNode && pathLibrary[i].endPoint == endNode)
        {
            return i;
        }
    }
    return -1;
}

double normalizeAngle360(double angle)
{
    angle = fmod(angle, 360.0);
    if (angle < 0)
    {
        angle += 360.0;
    }
    return angle;
    while (angle < 360)
        angle += 360;
    while (angle > 360)
        angle -= 360;

    return angle;
}

double normalizeAngle180(double angle)
{
    while (angle > 180.0)
        angle -= 360.0;
    while (angle < -180.0)
        angle += 360.0;
    return angle;
}
Path reversePath(const Path forwardPath)
{
    Path backwardPath;
    backwardPath.length = forwardPath.length;
    backwardPath.startPoint = forwardPath.endPoint;
    backwardPath.endPoint = forwardPath.endPoint;
    backwardPath.totalDistance_mm = forwardPath.totalDistance_mm;
    if (forwardPath.length < 3)
    {
        return backwardPath;
    }

    std::vector<double> forwardHeadings;
    std::vector<double> forwardDrives;
    for (int i = 0; i < forwardPath.length; ++i)
    {
        if (forwardPath.steps[i][0] == TURN_MODE)
        {
            forwardHeadings.push_back(forwardPath.steps[i][1]);
        }
        else
        {
            forwardDrives.push_back(forwardPath.steps[i][1]);
        }
    }

    int driveIndex = forwardDrives.size() - 1;
    int headingIndex = forwardDrives.size() - 1;
    int backwardStep = 0;

    // Loop through the reversed drives and turns
    while (driveIndex >= 0)
    {
        backwardPath.steps[backwardStep][0] = TURN_MODE;
        backwardPath.steps[backwardStep][1] = normalizeAngle360(forwardHeadings[headingIndex] + 180.0);
        backwardStep++;

        backwardPath.steps[backwardStep][0] = DRIVE_MODE;
        backwardPath.steps[backwardStep][1] = forwardDrives[driveIndex];
        backwardStep++;

        driveIndex--;
        headingIndex--;
    }

    // the final alignment has to be initial heading + 180 deg
    // this for symmetry
    backwardPath.steps[backwardStep][0] = TURN_MODE;
    backwardPath.steps[backwardStep][1] = normalizeAngle360(forwardHeadings[0] + 180.0);

    return backwardPath;
}

// driver functions

void initRobot()
{
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Calibrating Gyro");
    BrainInertial.calibrate();
    while (BrainInertial.isCalibrating())
    {
    }
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Calibration Compcete!");
    wait(1, sec);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);

    BrainInertial.setRotation(0, degrees); // Reset rotations (standard VEX)
    MotorLeft.setPosition(0, turns);
    MotorRight.setPosition(0, turns);
    MotorBox.setPosition(0, turns); // Assuming MotorBox is BoxOpener
    MotorBox.stop(hold);
    MotorLock.setPosition(0, turns);
    MotorLock.stop(hold);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Point robot NORTH ");
    Brain.Screen.newLine();
    Brain.Screen.print("and press L DOWN");

    while (!Controller.ButtonLDown.pressing())
    {
    }
    while (Controller.ButtonLDown.pressing())
    {
    }

    BrainInertial.setHeading(0, degrees);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Robot aligned with north");
    wait(2, sec);

    DriveMotors.resetPosition();
}

bool drive(double distance_mm)
{
    const double DRIVE_P = 0.5;
    const double SYNC_RATIO = 1;
    const double MIN_POWER = 300;                                      // we need to see what value can overcome friction
    const double MAX_POWER = 600 const double START_SLOW_DOWN = 500.0; // when to start slowing down
    const double MIN_OBSTACLE_DIST = 150.0;                            // when to fully stop if the obstacle is in this range
    DriveMotors.resetPosition();
    double initialHeading = BrainInertial.heading(degrees);

    while (turns_to_mm(DriveMotors.position(turns)) < distance_mm)
    {
        if (BumperSensor.pressing())
        {
            DriveMotors.stop(hold);
            Brain.Screen.print("obstacle!");
            return false;
        }
        // while(DistanceSensor.objectDistance(mm) < MIN_OBSTACLE_DIST) {
        //   DriveMotors.stop(hold);

        //   if (BumperSensor.pressing()) {
        //     return false;
        //   }
        // }

        // speed control
        double distanceToTravel = distance_mm - turns_to_mm(DriveMotors.position(turns));
        double basePower = MAX_POWER;

        if (distanceToTravel < START_SLOW_DOWN)
        {
            basePower = std::max(MIN_POWER, distanceToTravel * DRIVE_P);
        }

        // heading control
        double angleError = (initialHeading - BrainInertial.heading(degrees));
        angleError = normalizeAngle180(angleError);
        double correction = angleError * SYNC_RATIO * basePower / MAX_POWER;

        MotorLeft.spin(forward, std::max(0.0, basePower + correction), dps);
        MotorRight.spin(forward, std::max(0.0, basePower - correction), dps);
    }

    DriveMotors.stop(hold);
    return true;
}
void turn(int angle)
{ // i got this from the teacher files on learn
    int motorPower = 10;
    double startingAngle = BrainInertial.heading(degrees);

    MotorLeft.setVelocity(motorPower, dps);
    MotorRight.setVelocity(motorPower, dps);
    // for positive angle, rotates clockwise
    if (angle > 0)
    {
        MotorLeft.spin(forward);
        MotorRight.spin(reverse);
    }
    // for negative angle, rotates counterclockwise
    else
    {
        MotorLeft.spin(reverse);
        MotorRight.spin(forward);
    }
    while (abs(BrainInertial.heading(degrees) - startingAngle) < abs(angle))
    {
    }
    MotorLeft.stop();
    MotorRight.stop();
}

// void turnToHeading(double target_heading) {
//     const double TURN_TOLERANCE_DEG = 0.1; //find a value that actually works
//     const double SYNC_RATIO = 0.9275;

//     while (std::fabs(BrainInertial.heading(degrees) - target_heading) > TURN_TOLERANCE_DEG) {
//         wait(1, msec);
//         double current_heading = BrainInertial.heading(degrees);
//         double error = target_heading - current_heading;

//         normalizeAngle180(error); //make sure this actually works btw
//         //THIS DOES NOT FUCKING WORK BRO GET ME THE FUCK OUT

//         double motor_speed = turnSpeed(std::fabs(error));

//         if (error > 0) { // Clockwise
//             MotorLeft.spin(forward, motor_speed * SYNC_RATIO, dps);
//             MotorRight.spin(reverse, motor_speed, dps);
//         } else { // Counter-clockwise
//             MotorLeft.spin(reverse, motor_speed * SYNC_RATIO, dps);
//             MotorRight.spin(forward, motor_speed, dps);
//         }

//     }

//     MotorLeft.stop(hold);
//     MotorRight.stop(hold);
// } //without integral term

void turnToHeading(double target_heading)
{
    const double TURN_TOLERANCE_DEG = 1; // find a value that actually works
    const double SYNC_RATIO = 0.9275;
    const double TURN_CONSTANT = 10;
    const double INTEG_CONSTANT = 0.0005;
    double errorSum = 0;

    while (normalizeAngle180(BrainInertial.heading(degrees) - target_heading) > TURN_TOLERANCE_DEG)
    {
        wait(1, msec);
        double current_heading = BrainInertial.heading(degrees);
        double error = target_heading - current_heading;
        Brain.Screen.newLine();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("%f", BrainInertial.heading(degrees));
        // while (error > 180.0) error -= 360.0;
        // while (error < -180.0) error += 360.0;
        error = normalizeAngle180(error);

        errorSum += error;
        double motor_speed = TURN_CONSTANT * error + INTEG_CONSTANT * errorSum;

        if (error > 0)
        { // Clockwise
            MotorLeft.spin(forward, motor_speed * SYNC_RATIO, dps);
            MotorRight.spin(reverse, motor_speed, dps);
        }
        else
        { // Counter-clockwise
            MotorLeft.spin(reverse, motor_speed * SYNC_RATIO, dps);
            MotorRight.spin(forward, motor_speed, dps);
        }
    }

    MotorLeft.stop(hold);
    MotorRight.stop(hold);
}

// core logic

bool runSinglePath(const Path &pathToRun)
{
    int pathLength = pathToRun.length;

    for (int i = 0; i < pathLength; i++)
    {
        int mode = pathToRun.steps[i][0];

        if (mode == DRIVE_MODE)
        {
            double distance_mm = pathToRun.steps[i][1];
            bool success = drive(distance_mm);
            if (!success)
                return false;
        }
        else
        { // TURN_MODE
            double heading_deg = pathToRun.steps[i][1];
            turnToHeading(heading_deg);
        }
    }
    return true;
}

bool executeJourney(const std::vector<int> &waypoints, const std::vector<Path> &pathLibrary)
{ // do i need to pass by reference?? maybe just constant reference to prevent editing

    for (int i = 0; i < waypoints.size() - 1; i++)
    {
        int startPoint = waypoints[i];
        int endPoint = waypoints[i + 1];

        int pathIndice = findPath(startPoint, endPoint, pathLibrary); // does this need an ampersand
        if (pathIndice != -1)
        {
            bool success = runSinglePath(pathLibrary[pathIndice]);

            if (!success)
            {
                // return seems just like using break i wont even lie;
                return false;
            }
        }
        else
        {
            // ditto with above
            return false;
        }
    }

    return true;
}

void runTrainingMode(Path &pathToRecord)
{
    // we are forcing turn drive turn drive etc etc as the sequence of movements
    // this is to ensure

    // set the starting values;
    pathToRecord.length = 0;
    pathToRecord.totalDistance_mm = 0;
    DriveMotors.resetPosition();

    // keep track of the states
    int currentState = TURN_MODE;
    bool finished = false;
    double lastEncoderTurns = DriveMotors.position(turns);

    // keep track of the UI
    int lastShownMode = -1;
    int lastShownLength = -1;
    double lastShownDisplacement = -1; // do i really need this

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("--- TRAINING ---");
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("R Up: LOG COMMIT");
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("F Up: FINISH");

    while (!finished)
    {
        // first we need to check for the exit flag
        // thank you to tutorial ?? for the idea used here
        if (Controller.ButtonFUp.pressing() == 1)
        {
            finished = true;
        }

        if (!finished)
        {

            // we should only be updating the UI after a change is actually made
            // to prevent flickering

            // update  the mode we are in
            if (currentState != lastShownMode)
            {
                Brain.Screen.setCursor(2, 1);
                Brain.Screen.print("Mode: %s  ", (currentState == DRIVE_MODE ? "DRIVE" : "TURN"));
                lastShownMode = currentState;
            }

            // update the steps counts (length of the path)
            if (pathToRecord.length != lastShownLength)
            {
                Brain.Screen.setCursor(3, 1);
                Brain.Screen.print("Steps: %d   ", pathToRecord.length);
                lastShownLength = pathToRecord.length;
            }

            // now we show the displacement whether that be the angle turned or
            // distance travelled...

            if (currentState == DRIVE_MODE)
            {
                double dist = turns_to_mm(DriveMotors.position(turns) - lastEncoderTurns);
                if (std::fabs(dist - lastShownDisplacement) > 10)
                {
                    Brain.Screen.setCursor(4, 1);
                    Brain.Screen.print("DRIVE: %.1f mm", dist);
                    lastShownDisplacement = dist;
                }
            }
            else if (currentState == TURN_MODE)
            {
                double curHeading = BrainInertial.heading(degrees);
                if (std::fabs(curHeading - lastShownDisplacement) > 0.1)
                {
                    Brain.Screen.setCursor(4, 1);
                    Brain.Screen.print("TURN: %.1f deg", curHeading);
                    lastShownDisplacement = curHeading;
                }
            }

            if (Controller.ButtonRUp.pressing() == 1)
            { // push in the segment
                DriveMotors.stop(hold);

                if (currentState == DRIVE_MODE)
                {
                    double currentEncoderTurns = DriveMotors.position(turns);
                    double changeTurns = currentEncoderTurns - lastEncoderTurns;
                    double distance_mm = turns_to_mm(changeTurns);

                    pathToRecord.steps[pathToRecord.length][0] = DRIVE_MODE;
                    pathToRecord.steps[pathToRecord.length][1] = distance_mm;
                    pathToRecord.totalDistance_mm += distance_mm;
                    lastEncoderTurns = currentEncoderTurns;
                }
                else
                { // turn mode
                    double currentAbsoluteHeading = BrainInertial.heading(degrees);
                    pathToRecord.steps[pathToRecord.length][0] = TURN_MODE;
                    pathToRecord.steps[pathToRecord.length][1] = currentAbsoluteHeading;
                }
                pathToRecord.length++;

                // we need to swap the modes automatically
                if (currentState == DRIVE_MODE)
                {
                    currentState = TURN_MODE;
                }
                else
                {
                    currentState = DRIVE_MODE;
                }

                // force ui resets on these two values
                lastShownMode = -1;
                lastShownDisplacement = -10000;

                while (Controller.ButtonRUp.pressing())
                {
                }
                Brain.Screen.clearScreen();
                Brain.Screen.setCursor(1, 1);
            }

            else
            { // default case where I just allow the user to move around
                if (currentState == DRIVE_MODE)
                {
                    double drivePower = Controller.AxisA.position() * 4;
                    MotorLeft.spin(forward, drivePower, dps);
                    MotorRight.spin(forward, drivePower, dps);
                }
                else
                {
                    int turnPower = Controller.AxisC.position();
                    MotorLeft.spin(forward, turnPower, dps);
                    MotorRight.spin(reverse, turnPower, dps);
                }
            }
        }
    }

    double finalHeading = BrainInertial.heading(degrees);

    if (pathToRecord.steps[pathToRecord.length - 1][0] == TURN_MODE && pathToRecord.length != 0)
    {
        // force that the last turn value corresponds to the last drive segment
        // dont allow random rotations,
        // the reverse path function depends on this
        pathToRecord.steps[pathToRecord.length - 1][1] = finalHeading;
    }
    else
    {
        pathToRecord.steps[pathToRecord.length][0] = TURN_MODE;
        pathToRecord.steps[pathToRecord.length][1] = finalHeading;
        pathToRecord.length++;
    }
    DriveMotors.stop(hold);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Path generated yipee!");
}

std::string mainMenu()
{
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("E UP for training");
    Brain.Screen.newLine();
    Brain.Screen.print("E Down for Autonomous");
    Brain.Screen.newLine();
    Brain.Screen.print("F Up for exit");

    while (!Controller.ButtonEUp.pressing() && !Controller.ButtonEDown.pressing() && !Controller.ButtonFUp.pressing())
    {
    }
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    if (Controller.ButtonEUp.pressing())
    {
        return "TRAINING";
    }
    else if (Controller.ButtonEDown.pressing())
    {
        return "AUTONOMOUS";
    }
    else
    {
        return "EXIT";
    }
}

int main()
{
    // Initializing Robot Configuration. DO NOT REMOVE!
    vexcodeInit();
    // Begin project code
    Brain.Screen.setFont(mono12);
    const int timeToOpen = 15;       // 15 Seconds till siren
    const double openPosition = 5.5; // Position for box to go to when opened
    const double lockPosition = 6.5;

    // Initializing Robot Configuration. DO NOT REMOVE!
    vexcodeInit();
    initRobot();

    // std::vector<Path> pathLibrary = loadPathsFromFile();
    std::vector<Path> pathLibrary;

    double adjacencyMatrix[MAX_ROOMS + 1][MAX_ROOMS + 1] = {};
    createAdjacencyMatrix(pathLibrary, adjacencyMatrix);
    bool programExit = false;

    // Path p;
    // p.startPoint = 1;
    // p.endPoint = 2;
    // runTrainingMode(newPath);

    // Path p2 = p.startPoint  = 1;

    // )

    // Brain.programStop();

    while (!programExit)
    {
        std::string mode = mainMenu();

        if (mode == "TRAINING")
        {
            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("bkjawidwaidoijada");

            int start = getNumberInput("START ROOM:");
            int end = getNumberInput("ENDING ROOM:");
            if (start != -1 && end != -1)
            {
                Path newPath;
                newPath.startPoint = start;
                newPath.endPoint = end;
                runTrainingMode(newPath);

                pathLibrary.push_back(newPath);
                pathLibrary.push_back(reversePath(newPath));
                createAdjacencyMatrix(pathLibrary, adjacencyMatrix); // just in autonomous node prolly
            }
        }
        else if (mode == "AUTONOMOUS")
        {
            int start = getNumberInput("START ROOM:");
            int end = getNumberInput("ENDING ROOM:");
            if (start != -1 && end != -1)
            {
                std::vector<int> waypoints = shortestPath(start, end, adjacencyMatrix);

                if (!waypoints.empty())
                {
                    Brain.Screen.print("running the path");
                    bool success = executeJourney(waypoints, pathLibrary);

                    if (success)
                    {
                        openBox(openPosition, lockPosition, timeToOpen);
                    }
                    else
                    {
                        Brain.Screen.clearScreen();
                        Brain.Screen.setCursor(1, 1);
                        Brain.Screen.print("MISSION FAILED / ABORTED");
                    }
                }
                else
                {
                    Brain.Screen.clearScreen();
                    Brain.Screen.setCursor(1, 1);
                    Brain.Screen.print("no path found");
                }
            }
        }
        else
        { // mode is shutdown
            //(pathLibrary);
            programExit = true;
        }
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        wait(1000, msec);
    }

    Brain.Screen.print("program was shut down :(");
    wait(5000, msec);
    Brain.programStop();
}
