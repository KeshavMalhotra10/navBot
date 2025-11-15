#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <sstream>

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
#include <limits>   //allows for a proper shortest path implementation
#include <stdio.h>  // Required for sprintf
#include <string.h> // Required for strlen
#include <sstream>
// Allows for easier use of the VEX Library
using namespace vex;

// global var and constant
const int MAX_ROOMS = 5; // will make this bigger later
const int WHEEL_CIRCUMFERENCE_MM = 200;
const int DRIVE_MODE = 0;
const int TURN_MODE = 1;
const int FIXED_POINT_SCALE = 100;
// needs to be a c style string
const char *PATH_FILE = "robotPaths.txt";
motor_group DriveMotors(MotorLeft, MotorRight);

class Path
{
public:
    static const int MAX_STEPS = 5;
    int startPoint = -1;
    int endPoint = -1;
    double steps[MAX_STEPS][2] = {0};
    int length = 0;
    double totalDistance_mm = 0;
};

bool savePathsToFile(const std::vector<Path> &pathLibrary)
{
    if (!Brain.SDcard.isInserted())
        return false;
    if (pathLibrary.empty())
    {
        uint8_t z[] = "0\n";
        Brain.SDcard.savefile("robotPaths.txt", z, sizeof(z) - 1);
        return true;
    }
    char buffer[100];
    sprintf(buffer, "%d\n", (int)pathLibrary.size());
    Brain.SDcard.savefile("robotPaths.txt", (uint8_t *)buffer, strlen(buffer));

    for (int i = 0; i < pathLibrary.size(); i++)
    {

        Path p = pathLibrary[i];

        sprintf(buffer, "%d %d %d %d\n",
                p.startPoint,
                p.endPoint,
                (int)((double)p.totalDistance_mm * FIXED_POINT_SCALE),
                p.length);
        Brain.SDcard.appendfile("robotPaths.txt", (uint8_t *)buffer, strlen(buffer));

        for (int j = 0; j < p.length; j++)
        {
            // Do the same for the steps.
            sprintf(buffer, "%d %d\n",
                    (int)p.steps[j][0],                                // Mode is already an int, but cast is safe
                    (int)((double)p.steps[j][1] * FIXED_POINT_SCALE)); // <-- THE CHANGE
            Brain.SDcard.appendfile("robotPaths.txt", (uint8_t *)buffer, strlen(buffer));
        }
    }
    return true;
}

void deleteSavedPaths()
{
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("--- Resetting Data ---");
    wait(500, msec);

    if (!Brain.SDcard.isInserted())
    {
        Brain.Screen.print("ERR: NO SDCARD");
        wait(2, sec);
        return;
    }

    // Check if the file exists before trying to "delete" it
    if (Brain.SDcard.exists("robotPaths.txt"))
    {
        // To "delete" the file, we overwrite it with an empty but valid state.
        // This file content means "0 paths".
        uint8_t empty_file_content[] = "0\n";

        // The -1 is important to avoid writing the string's null terminator '\0'
        bool success = Brain.SDcard.savefile("robotPaths.txt", empty_file_content, sizeof(empty_file_content) - 1);

        if (success)
        {
            Brain.Screen.print("Saved paths have been deleted.");
        }
        else
        {
            Brain.Screen.print("ERROR: Could not write to SD Card!");
        }
    }
    else
    {
        Brain.Screen.print("No saved paths found. Nothing to delete.");
    }

    wait(2, sec); // Give the user time to read the message
}

bool loadPathsFromFile(std::vector<Path> &pathLibrary)
{

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Loading File...");

    if (!Brain.SDcard.isInserted())
    {
        Brain.Screen.print("E: no sd card");
        wait(3000, msec);
        return false;
    }
    else if (!Brain.SDcard.exists(PATH_FILE))
    {
        Brain.Screen.print("E: no file found");
        wait(2000, msec);
        return false;
    }

    int fileSize = Brain.SDcard.size(PATH_FILE);
    if (fileSize == 0)
    {
        return true;
        pathLibrary.clear();
    }

    uint8_t *buffer = new uint8_t[fileSize + 1];
    if (!buffer)
    {
        Brain.Screen.print("E: NOT ENOUGH MEM");
        wait(3000, msec);
        return false;
    }

    Brain.SDcard.loadfile(PATH_FILE, buffer, fileSize);
    buffer[fileSize] = '\0';
    pathLibrary.clear();

    char *readPoint = (char *)buffer;
    int totalPaths = 0;
    int offset = 0;
    int itemsRead = 0;

    bool success = true;

    itemsRead = sscanf(readPoint, "%d%n", &totalPaths, &offset);
    if (itemsRead != 1)
    {
        Brain.Screen.print("E: BAD FILE");
        success = false;
    }
    readPoint += offset;

    for (int i = 0; i < totalPaths && success; i++)
    {
        Path pathToAdd;
        int tempDist = 0;
        int tempStepMode = 0;
        int tempStepVal = 0;

        itemsRead = sscanf(readPoint, "%d %d %d %d%n",
                           &pathToAdd.startPoint, &pathToAdd.endPoint,
                           &tempDist, &pathToAdd.length, &offset);

        if (itemsRead != 4)
        {
            Brain.Screen.print("E: CORRUPT PATH");
            success = false;
        }
        else
        {
            readPoint += offset;
            pathToAdd.totalDistance_mm = (double)tempDist / FIXED_POINT_SCALE;

            for (int j = 0; j < pathToAdd.length; j++)
            {
                sscanf(readPoint, "%d %d%n", &tempStepMode, &tempStepVal, &offset);
                readPoint += offset;
                pathToAdd.steps[j][0] = tempStepMode;
                pathToAdd.steps[j][1] = (double)tempStepVal / FIXED_POINT_SCALE;
            }
            pathLibrary.push_back(pathToAdd);
        }
    }

    delete[] buffer;

    if (success)
    {
        Brain.Screen.print("Load Succeeded!");
        wait(1500, msec);
        return true;
    }
    else
    {
        wait(3000, msec);
        return false;
    }
}

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
        double dist = pathLibrary[i].totalDistance_mm;
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
Path reversePath(Path forwardPath)
{
    Path backwardPath;
    backwardPath.length = forwardPath.length;
    backwardPath.startPoint = forwardPath.endPoint;
    backwardPath.endPoint = forwardPath.startPoint;
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

    // while(!Controller.ButtonLDown.pressing()) {
    // }
    // while(Controller.ButtonLDown.pressing()) {
    // }

    BrainInertial.setHeading(0, degrees);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Robot aligned with north");
    wait(2, sec);

    DriveMotors.resetPosition();
}

// bool drive(double distance_mm) {
//   const double DRIVE_P = 0.5;
//   const double SYNC_RATIO = 1;
//   const double MIN_POWER = 300;    //we need to see what value can overcome friction
//   const double MAX_POWER = 600;
//   const double START_SLOW_DOWN = 500.0;  //when to start slowing down
//   const double MIN_OBSTACLE_DIST = 150.0;   //when to fully stop if the obstacle is in this range
//   DriveMotors.resetPosition();
//   double initialHeading = BrainInertial.heading(degrees);

//   while(turns_to_mm(DriveMotors.position(turns)) < distance_mm) {
//     if(BumperSensor.pressing()) {
//       DriveMotors.stop(hold);
//       Brain.Screen.print("obstacle!");
//       return false;
//     }
//     while(DistanceSensor.objectDistance(mm) < MIN_OBSTACLE_DIST) {
//       DriveMotors.stop(hold);
//       if (BumperSensor.pressing()) {
//         return false;
//       }
//     }

//     //speed control
//     double distanceToTravel = distance_mm - turns_to_mm(DriveMotors.position(turns));
//     double basePower = MAX_POWER;

//     if(distanceToTravel < START_SLOW_DOWN) {
//       basePower = std::max(MIN_POWER, distanceToTravel * DRIVE_P);

//     }

//     //heading control
//     double angleError = (initialHeading - BrainInertial.heading(degrees));
//     angleError = normalizeAngle180(angleError);
//     double correction = angleError * SYNC_RATIO * basePower/MAX_POWER;

//     MotorLeft.spin(forward, std::max(0.0,basePower + correction), dps);
//     MotorRight.spin(forward, std::max(0.0,basePower - correction), dps);
//   }

//   DriveMotors.stop(hold);
//   return true;
// }

bool drive(double distance_mm)
{
    const double DISTANCE_P = 0.05;
    const double HEADING_P = 0.005;
    const double HEADING_I = 0.0000;
    const double MIN_POWER = 30;
    const double MAX_POWER = 70;
    const double START_SLOWDOWN_MM = 200;
    const double MIN_OBSTACLE_DISTANCE_MM = 100;

    double headingErrorSum = 0;
    DriveMotors.resetPosition();
    double initialHeading = BrainInertial.heading(degrees);

    while (turns_to_mm(DriveMotors.position(turns)) < distance_mm)
    {
        if (BumperSensor.pressing())
        {
            DriveMotors.stop(hold);
            Brain.Screen.print("obstacle!");
            wait(2000, msec);
            return false;
        }
        while (DistanceSensor.objectDistance(mm) < MIN_OBSTACLE_DISTANCE_MM)
        {
            DriveMotors.stop(hold);
            if (BumperSensor.pressing())
            {
                Brain.Screen.print("obstacle!");
                wait(2000, msec);
                return false;
            }
        }

        // control for base power (distance controller)
        double distanceToTravel = distance_mm - turns_to_mm(DriveMotors.position(turns));
        double basePower = MAX_POWER;

        if (distanceToTravel < START_SLOWDOWN_MM)
        {
            basePower = std::max(MIN_POWER, distanceToTravel * DISTANCE_P);
        }

        // heading control
        double headingError = normalizeAngle180((normalizeAngle180(initialHeading) - normalizeAngle180(BrainInertial.heading(degrees))));

        headingErrorSum += headingError;

        double correction = headingError * HEADING_P + headingErrorSum * HEADING_I;
        double leftPower = basePower * (1 + correction);
        double rightPower = basePower * (1 - correction);

        leftPower = std::min(100.0, std::max(0.0, leftPower));
        rightPower = std::min(100.0, std::max(0.0, rightPower));

        MotorLeft.spin(forward, std::max(0.0, leftPower), percent);
        MotorRight.spin(forward, std::max(0.0, rightPower), percent);
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

bool deletePathByRoom(int start, int end, std::vector<Path> &pathLibrary, double (&adjacencyMatrix)[MAX_ROOMS + 1][MAX_ROOMS + 1])
{
    for (int i = 0; i < pathLibrary.size(); i++)
    {
        Path p = pathLibrary[i];
        if (p.startPoint == start && p.endPoint == end)
        {
            pathLibrary.erase(pathLibrary.begin() + i);
            createAdjacencyMatrix(pathLibrary, adjacencyMatrix);
            return true;
        }
    }

    return false;
}

void turnToHeading(double target_heading)
{
    const double TURN_TOLERANCE_DEG = 1; // find a value that actually works
    // const double SYNC_RATIO = 0.9275;
    const double TURN_CONSTANT = 10;
    const double INTEG_CONSTANT = 0.01;
    const double INTEG_MAX = 50;
    double errorSum = 0;

    double current_heading = BrainInertial.heading(degrees);
    double error = normalizeAngle180(normalizeAngle180(target_heading) - normalizeAngle180(current_heading));

    while (std::fabs(error) > TURN_TOLERANCE_DEG)
    {
        wait(1, msec);
        current_heading = BrainInertial.heading(degrees);
        error = (target_heading - current_heading);
        Brain.Screen.newLine();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("%lf", BrainInertial.heading(degrees));

        error = normalizeAngle180(normalizeAngle180(target_heading) - normalizeAngle180(current_heading));

        //  errorSum += error;
        //  if(errorSum > INTEG_MAX)
        //   errorSum = INTEG_MAX;
        //  else if(errorSum < -INTEG_MAX)
        //   errorSum = -1 * INTEG_MAX;
        double motor_speed = TURN_CONSTANT * error + INTEG_CONSTANT * errorSum;

        if (error < 0)
        {                                              // Clockwise
            MotorLeft.spin(forward, motor_speed, dps); // note i got rid of sync ratio = 0.9275, maybe add it back idfk
            MotorRight.spin(reverse, motor_speed, dps);
        }
        else
        { // Counter-clockwise
            MotorLeft.spin(reverse, motor_speed, dps);
            MotorRight.spin(forward, motor_speed, dps);
        }
    }

    MotorLeft.stop(hold);
    MotorRight.stop(hold);
}
void relativeTurn(double angle_degrees)
{
    const double TURN_TOLERANCE_DEG = 1.0;
    const double TURN_CONSTANT = 10.0;
    const double INTEG_CONSTANT = 0.02;
    const double INTEG_MAX = 50.0;

    double errorSum = 0.0;
    BrainInertial.setRotation(0, degrees);

    double error = angle_degrees;

    while (std::fabs(error) > TURN_TOLERANCE_DEG)
    {
        wait(1, msec);
        double rotation_so_far = BrainInertial.rotation(degrees);
        error = angle_degrees - rotation_so_far;

        errorSum += error;
        if (errorSum > INTEG_MAX)
        {
            errorSum = INTEG_MAX;
        }
        else if (errorSum < -INTEG_MAX)
        {
            errorSum = -1 * INTEG_MAX;
        }

        double motor_speed = TURN_CONSTANT * error + INTEG_CONSTANT * errorSum;

        if (error > 0)
        {
            MotorLeft.spin(forward, motor_speed, dps);
            MotorRight.spin(reverse, motor_speed, dps);
        }
        else
        {
            MotorLeft.spin(reverse, -motor_speed, dps);
            MotorRight.spin(forward, -motor_speed, dps);
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
        { // TURN mode !!902903109i309i21309i21dsadoijaoijda

            double target_heading = pathToRun.steps[i][1];
            double current_heading = BrainInertial.heading(degrees);
            double angleToTurn = normalizeAngle180(target_heading - current_heading);
            relativeTurn(angleToTurn);
        }
    }
    return true;
}

bool executeJourney(const std::vector<int> &waypoints, const std::vector<Path> &pathLibrary)
{ // do i need to pass by reference?? maybe just constant reference to prevent editing

    for (int i = 0; i < waypoints.size() - 1; i++)
    {
        wait(500, msec);
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
    const double INTEGRAL_LIMIT = 10;
    double headingErrorSum = 0.0; // Integral term for straightness
    double targetHeading = BrainInertial.heading(degrees);
    // set the starting values;
    pathToRecord.length = 0;
    pathToRecord.totalDistance_mm = 0;
    DriveMotors.resetPosition();
    double previous_encoder_turns = 0.0;

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
        wait(1, msec);
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
                    targetHeading = BrainInertial.heading(degrees);
                    headingErrorSum = 0;
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
                    double drivePower = 0;

                    if (Controller.AxisA.position() > 50)
                    {
                        drivePower = 50;
                    }
                    else if (Controller.AxisA.position() < -50)
                    {
                        drivePower = -50;
                    }
                    double sign = (drivePower > 0) ? 1.0 : -1;
                    const double HEADING_CORRECTION_P = 0.03;
                    const double HEADING_CORRECTION_I = 0;
                    double deltaTurns = DriveMotors.position(turns) - previous_encoder_turns;
                    double curHeading = BrainInertial.heading(degrees);
                    double headingError = targetHeading - curHeading;

                    headingError = normalizeAngle180(headingError); // do i even need this
                    headingErrorSum += headingError * deltaTurns;
                    headingErrorSum = std::min(INTEGRAL_LIMIT, std::max(-INTEGRAL_LIMIT, headingErrorSum));
                    double correction = headingError * HEADING_CORRECTION_P + headingErrorSum * HEADING_CORRECTION_I;
                    if (correction > 0.5)
                        correction = 0.5;
                    if (correction < -0.5)
                        correction = -0.5;

                    MotorLeft.spin(forward, drivePower * (1.0 + correction * sign), percent);
                    MotorRight.spin(forward, drivePower * (1.0 - correction * sign), percent);
                    previous_encoder_turns = DriveMotors.position(turns);
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
    Brain.Screen.newLine();
    Brain.Screen.print("F Down to delete path");

    while (!Controller.ButtonEUp.pressing() && !Controller.ButtonEDown.pressing() && !Controller.ButtonFUp.pressing() && !Controller.ButtonFDown.pressing())
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
    else if (Controller.ButtonFUp.pressing())
    {
        return "EXIT";
    }
    else
    {
        return "DELETE";
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

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    std::vector<Path> pathLibrary;
    bool pathsFound = loadPathsFromFile(pathLibrary);

    wait(1000, msec);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);

    if (pathsFound)
    {
        Brain.Screen.print("loaded % paths", pathLibrary.size());
        Brain.Screen.setCursor(1, 1);

        Brain.Screen.print("Loaded %d paths!", pathLibrary.size());
    }
    else
    {
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("no saved data unfortunately");
    }
    wait(5000, msec);

    // Brain.programStop();

    double adjacencyMatrix[MAX_ROOMS + 1][MAX_ROOMS + 1] = {};
    createAdjacencyMatrix(pathLibrary, adjacencyMatrix);
    bool programExit = false;

    while (!programExit)
    {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        std::string mode = mainMenu();

        if (mode == "TRAINING")
        {
            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(1, 1);

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
        else if (mode == "DELETE")
        {
            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(1, 1);
            int start = getNumberInput("DELETE: START ROOM:");
            int end = getNumberInput("DELETE: END ROOM:");
            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(1, 1);

            bool deleted = deletePathByRoom(start, end, pathLibrary, adjacencyMatrix);
            if (deleted)
            {
                Brain.Screen.print("path deleted");
            }
            else
            {
                Brain.Screen.print("path doesn't exist");
            }

            wait(2000, msec);
        }
        else
        { // mode is shutdown
            savePathsToFile(pathLibrary);
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
