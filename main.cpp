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

void vexcodeInit()
{

    // Initializing random seed.
    initializeRandomSeed();
}

#pragma endregion VEXcode Generated Robot Configuration
// Include the IQ Library
#include "iq_cpp.h"
#include <string>
#include <vector>
#include <limits> //allows for a proper shortest path implementation

#include <stdio.h> // Required for sprintf

using namespace vex;
// ------------------------------------------
// GLOBAL CONSTANTS
// ------------------------------------------
const int MAX_ROOMS = 10;
const int WHEEL_CIRCUMFERENCE_MM = 200;
const int DRIVE_MODE = 0;
const int TURN_MODE = 1;
const int FIXED_POINT_SCALE = 100;
const char *PATH_FILE = "robotPaths.txt"; // needs to be a c style string

// Motor group combining left and drive motor to run together
motor_group DriveMotors(MotorLeft, MotorRight);

// ------------------------------------------
// PATH STRUCTURE
// ------------------------------------------
// Represents a full path from one location to another

struct Path
{
    static const int MAX_STEPS = 10;
    int startPoint = 0;
    int endPoint = 0;
    float steps[MAX_STEPS][2] = {0};
    int length = 0;
    float totalDistance_mm = 0;
};

// ------------------------------------------
// FILE I/O FUNCTIONS
// ------------------------------------------
/*
the following code was written extensively with the help of chatGPT
permission was received from the MTE 121 professor.

1. Saves all paths to an SD card file
2. pathLibrary vectors contains all the recorded paths
3. it returns true if save was sucessfuly, else false

*/
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
                (int)((float)p.totalDistance_mm * FIXED_POINT_SCALE),
                p.length);
        Brain.SDcard.appendfile("robotPaths.txt", (uint8_t *)buffer, strlen(buffer));

        for (int j = 0; j < p.length; j++)
        {

            sprintf(buffer, "%d %d\n",
                    (int)p.steps[j][0],
                    (int)((float)p.steps[j][1] * FIXED_POINT_SCALE));
            Brain.SDcard.appendfile("robotPaths.txt", (uint8_t *)buffer, strlen(buffer));
        }
    }
    return true;
}

/*the following code was written extensively with the help of chatGPT
permission was received from the MTE 121 professor.

Loads all paths from SD Card file into path library
the pathLibrary vector is used to store the loaded paths
true is returned if load was successful, else false
*/

bool loadPathsFromFile(std::vector<Path> &pathLibrary)
{
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Loading File...");
    Brain.Screen.newLine();
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
        pathLibrary.clear();
        return true;
    }

    const int BUFFER_SIZE = 2048;

    if (fileSize >= BUFFER_SIZE)
    {
        Brain.Screen.print("E: FILE TOO LARGE");
        wait(3000, msec);
        return false;
    }

    static uint8_t buffer[BUFFER_SIZE];

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
        Brain.Screen.print("E: BAD FILE HEADER");
        success = false;
    }

    if (success)
    {
        readPoint += offset;

        for (int i = 0; i < totalPaths && success; i++)
        {
            Path pathToAdd;
            int tempDist = 0;

            itemsRead = sscanf(readPoint, "%d %d %d %d%n",
                               &pathToAdd.startPoint, &pathToAdd.endPoint,
                               &tempDist, &pathToAdd.length, &offset);
            if (itemsRead != 4)
            {
                Brain.Screen.print("E: CORRUPT PATH HDR");
                success = false;
            }
            else
            {
                readPoint += offset;
                pathToAdd.totalDistance_mm = (float)tempDist / FIXED_POINT_SCALE;

                for (int j = 0; j < pathToAdd.length && success; j++)
                {
                    int tempStepMode = 0;
                    int tempStepVal = 0;
                    itemsRead = sscanf(readPoint, "%d %d%n", &tempStepMode, &tempStepVal, &offset);

                    if (itemsRead != 2)
                    {
                        Brain.Screen.print("E: CORRUPT STEP DATA");
                        success = false;
                    }
                    else
                    {
                        readPoint += offset;
                        pathToAdd.steps[j][0] = tempStepMode;
                        pathToAdd.steps[j][1] = (float)tempStepVal / FIXED_POINT_SCALE;
                    }
                }

                if (success)
                {
                    pathLibrary.push_back(pathToAdd);
                }
            }
        }
    }

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
// ------------------------------------------
// BOX CONTROL FUNCTIONS
// ------------------------------------------

/*
Wait for user to press touch sensor
If user does not press button within specified time, alarm goes off
waitTime paramater is the specified time



*/
void pressToOpen(int waitTime)
{
    // start a timer
    float initialTime = Brain.Timer.value();

    // wait for the touch sensor to be pressed, or alarm to go off
    while (!TouchSensor.pressing() && (Brain.Timer.value() - initialTime) < waitTime)
    {
    }
    // If specified time has passed, play the siren sound as an alarm
    if (!TouchSensor.pressing())
    {
        while (!TouchSensor.pressing())
        {
            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("Please press the button!");
            Brain.playSound(siren);
            wait(1, seconds);
        }
    }
    // Wait until the touch sensor is no longer be pressed
    while (TouchSensor.pressing())
    {
    }
}
/*
waits until user presses and releases the touch sensor
*/
void pressToClose()
{
    // wait for press
    while (!TouchSensor.pressing())
    {
    }
    // wait for release
    while (TouchSensor.pressing())
    {
    }
}
/*
1. Opens delivery box, wait for user touch, then closes it
2. openPosition paramter is the position in cm for the box lid
3. lockPosition paramter is the position in cm for lock mechanism
4. waitTime paramater is the time to wait before the alarm sounds
*/
void openBox(float openPosition, float lockPosition, int waitTime)
{
    const float Turns_to_Cm = 20.0; // Motor turns to centimeters conversion

    // Display message
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Open Box!");

    // wait for user to press the touch sensor
    pressToOpen(waitTime);

    // unlock the box
    MotorLock.spin(forward, 7, percent);
    while (MotorLock.position(turns) * Turns_to_Cm < lockPosition)
    {
        wait(20, msec);
    }
    // open the lid of the box
    MotorLock.stop(hold);
    MotorBox.spin(forward, 7, percent);
    while (MotorBox.position(turns) * Turns_to_Cm < openPosition)
    {
        wait(20, msec);
    }
    MotorBox.stop(hold);
    // wait for user to press touch sensor
    pressToClose();

    // Close the box lid
    MotorBox.spin(reverse, 7, percent);
    while (MotorBox.position(turns) > 0)
    {
    }
    MotorBox.stop(hold);

    // Lock box
    MotorLock.spin(reverse, 7, percent);
    while (MotorLock.position(turns) > 0)
    {
    }
    MotorLock.stop(hold);
    Brain.Screen.clearScreen();
}

// ------------------------------------------
// USER INTERFACE FUNCTIONS
// ------------------------------------------

/*
1. User selects number using controller buttons
2. prompt paramater is the text shown to the user
3. return a number, (1-MAX_ROOMS), if user cancels return -1
*/
int getNumberInput(const std::string prompt)
{
    int currentSelection = 1; // currently selected number
    int lastSelection = 1;    // previously selected number

    // Initial display
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

    // wait until user either confirm or cancels
    while (Controller.ButtonRUp.pressing() == 0 && Controller.ButtonLUp.pressing() == 0)
    {
        // display the new selection
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
        // Selection number increases
        if (Controller.ButtonEUp.pressing() == 1)
        {
            currentSelection++;

            ` //if selection exceeds max_rooms, then go back to 1
                if (currentSelection > MAX_ROOMS)
            {
                currentSelection = 1; // wrap around
            }
            // wait until the button is released
            while (Controller.ButtonEUp.pressing() == 1)
            {
            }
        }
        // selection number decreases
        else if (Controller.ButtonEDown.pressing() == 1)
        {
            currentSelection--;

            // if user tries to go less than 1, go the to last path
            if (currentSelection < 1)
            {
                currentSelection = MAX_ROOMS; // wrap around
            }
            // wait until the button is released
            while (Controller.ButtonEDown.pressing() == 1)
            {
            }
        }
        wait(5, msec);
    }
    // cancel button pressed, return -1
    if (Controller.ButtonLUp.pressing())
    {
        currentSelection = -1;
    }
    // ensure all buttons are released to prevent incorrect navigation
    while (Controller.ButtonRUp.pressing() || Controller.ButtonLUp.pressing())
    {
    }

    return currentSelection;
}

/*
return the menu mode selected
options are
1. training
2. autonomous
3. exit
4. delete path
*/

// ------------------------------------------
// MAIN MENU
// ------------------------------------------
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

    // while none of the menu buttons are being pressed
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

// ------------------------------------------
// PATHFINDING FUNCTIONS
// ------------------------------------------

/*
Adjacency matrix is made taking the path library to develop shortest path
Matrix stores the distances between connected rooms
pathLibrary paramater is a vector of recorded paths
adjacenyMatrix paramter ouputs a 2d array where
[i][j] is the distance from room i to j, returns (-1) if no direct path
*/
void createAdjacencyMatrix(const std::vector<Path> &pathLibrary, float (&adjacencyMatrix)[MAX_ROOMS + 1][MAX_ROOMS + 1])
{
    // resets the array, no direct path, initialized to -1
    for (int i = 0; i <= MAX_ROOMS; i++)
    {

        for (int j = 0; j <= MAX_ROOMS; j++)
        {
            adjacencyMatrix[i][j] = -1;
        }
    }
    // adds each path to matrix
    for (int i = 0; i < pathLibrary.size(); i++)
    {
        int start = pathLibrary[i].startPoint;
        int end = pathLibrary[i].endPoint;
        float dist = pathLibrary[i].totalDistance_mm;
        adjacencyMatrix[start][end] = dist;
    }
}

/*
 - Finds shortest path using Djikstra's algorithm
 - start paramater is the starting room number
 - end paramater is ending room number
 - adjacencyMatrix paramter is a matrix of the distances between connecting rooms
 - return vector of room numbers representing the path
*/
std::vector<int> shortestPath(int start, int end, const float (&adjacencyMatrix)[MAX_ROOMS + 1][MAX_ROOMS + 1])
{

    const float INF = std::numeric_limits<float>::infinity();

    float distances[MAX_ROOMS + 1];
    int came_from[MAX_ROOMS + 1];
    bool visited[MAX_ROOMS + 1];

    for (int i = 1; i <= MAX_ROOMS; i++)
    {

        distances[i] = INF;
        came_from[i] = 0;
        visited[i] = false;
    }

    distances[start] = 0;
    for (int i = 1; i <= MAX_ROOMS; i++)
    {
        float minDist = INF;
        int closestPoint = -1; //-1 indicates not found yet
        // this part finds the node with the least distance that hasnt been visited
        for (int j = 1; j <= MAX_ROOMS; j++)
        {
            if (!visited[j] && distances[j] < minDist)
            {
                minDist = distances[j];
                closestPoint = j;
            }
        }

        // closestPoint = -1 would mean we are done exploring
        // otherwise we explore from the closest point
        if (closestPoint != -1)
        {
            visited[closestPoint] = true;

            for (int neighbour = 1; neighbour <= MAX_ROOMS; neighbour++)
            {
                // go through every neighbour for this point
                float distNeighbour = adjacencyMatrix[closestPoint][neighbour];

                if (distNeighbour != -1 && !visited[neighbour])
                {
                    // if there is a path to this point
                    // and it has not been explored yet

                    float potentialPathDist = distances[closestPoint] + distNeighbour;
                    if (potentialPathDist < distances[neighbour])
                    {
                        // if we have a better path to point (shorter distance)
                        // update our distance to the point
                        distances[neighbour] = potentialPathDist;

                        // update the best way to arrive to this point
                        came_from[neighbour] = closestPoint;
                    }
                }
            }
        }
    }

    /*
    get the literal path
    go backwards from the ending room to the start
    */
    std::vector<int> path;

    if (distances[end] < INF)
    {
        int currentPosition = end;

        while (currentPosition != 0)
        {
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

// ------------------------------------------
// HELPER/UTILITY FUNCTIONS
// -----------------------------------------

/*
1. Motor turns converted to millimeters travelled
2. numTurns paramater is the Number of motor rotations
3. return the Distance in millimeters
*/
float turns_to_mm(float numTurns)
{
    return numTurns * WHEEL_CIRCUMFERENCE_MM;
}

float mm_to_turns(float distance_mm)
{
    return distance_mm / WHEEL_CIRCUMFERENCE_MM;
}

int findPath(int startNode, int endNode, const std::vector<Path> &pathLibrary)
{
    // go through the library and find a path
    // with matching starting and ending nodes
    for (int i = 0; i < pathLibrary.size(); ++i)
    {
        if (pathLibrary[i].startPoint == startNode && pathLibrary[i].endPoint == endNode)
        {
            return i;
        }
    }
    return -1;
}

// changes an angle to be between 0, 360
float normalizeAngle360(float angle)
{
    while (angle < 360)
        angle += 360;
    while (angle > 360)
        angle -= 360;

    return angle;
}

// changes an angle to be between 0, 180
float normalizeAngle180(float angle)
{
    while (angle > 180.0)
        angle -= 360.0;
    while (angle < -180.0)
        angle += 360.0;
    return angle;
}

/*
returns the reverse of an input path
cuts down training time, robot can navigate backwards along a learned route
forwardPath paramter, the original path
return the reversed path
*/
Path reversePath(Path forwardPath)
{
    Path backwardPath;
    backwardPath.length = forwardPath.length;
    // swaps start and end points
    backwardPath.startPoint = forwardPath.endPoint;
    backwardPath.endPoint = forwardPath.startPoint;
    backwardPath.totalDistance_mm = forwardPath.totalDistance_mm;
    if (forwardPath.length < 3)
    {
        // doesn't make sense to reverse paths that are only a turn
        return backwardPath;
    }

    std::vector<float> forwardHeadings;
    std::vector<float> forwardDrives;
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
    } // separates the input path into turn headings and distances

    int driveIndex = forwardDrives.size() - 1;
    int headingIndex = forwardDrives.size() - 1;
    int backwardStep = 0;

    // Loop through the reversed drives and turns
    // and reconstruct the backward path
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

    // the final alignment turn value
    // has to be initial heading + 180 deg
    // this for symmetry
    backwardPath.steps[backwardStep][0] = TURN_MODE;
    backwardPath.steps[backwardStep][1] = normalizeAngle360(forwardHeadings[0] + 180.0);

    return backwardPath;
}

// ------------------------------------------
// INITIALIZE ROBOT
// ------------------------------------------

void initRobot()
{
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("now we calibrate the gyro");
    BrainInertial.calibrate();
    while (BrainInertial.isCalibrating())
    {
    }
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Calibration is done");
    wait(1, sec);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);

    BrainInertial.setRotation(0, degrees);
    MotorLeft.setPosition(0, turns);
    MotorRight.setPosition(0, turns);
    MotorBox.setPosition(0, turns);
    MotorBox.stop(hold);
    MotorLock.setPosition(0, turns);
    MotorLock.stop(hold);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Point robot NORTH ");

    /*
    allow the user to turn the robot
    using the joystick and align it with
    north
    */
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Use joystick to turn");
    Brain.Screen.newLine();
    Brain.Screen.print("Press FUp to align");
    while (!Controller.ButtonFUp.pressing())
    {
        int turnPower = Controller.AxisC.position();
        MotorLeft.spin(forward, turnPower, dps);
        MotorRight.spin(reverse, turnPower, dps);
    }

    Brain.Screen.newLine();
    Brain.Screen.print("and press L down");
    // sets the current heading to north which is 0 degrees
    BrainInertial.setHeading(0, degrees);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Robot aligned with north!!");
    wait(2, sec);

    // reset motor positions at end, for accuracy
    DriveMotors.resetPosition();
}

// ------------------------------------------
// MOVEMENT FUNCTIONS
// ------------------------------------------

/*
1. Robot Drives forward specified distance with obstacle detection
2. Proportional control used to maintain straight heading
3. Completely stops if obstacle is detected
4. distance_mm paramater is the specified distance to drive
5. returns true if this is successful, else false if obstacle detected
*/

bool drive(float distance_mm)
{
    const float MAX_POWER = 70.0;
    const float MIN_POWER = 20.0;

    // from this distance the robot's speed linearly decreases from max to min power
    const float SLOWDOWN_DISTANCE_MM = 100.0;

    // robot stops within this distance from an object
    const float MIN_OBSTACLE_DISTANCE_MM = 250.0;

    const float HEADING_P = 0.01; // determined emprically

    DriveMotors.resetPosition();
    float initialHeading = BrainInertial.heading(degrees);

    while (turns_to_mm(DriveMotors.position(turns)) < distance_mm)
    {
        // collisions should take us out of the function
        if (BumperSensor.pressing())
        {
            DriveMotors.stop(hold);
            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("Obstacle Detected! oh noo!!!!");
            wait(2000, msec);
            return false;
        }

        float distanceRemaining = distance_mm - turns_to_mm(DriveMotors.position(turns));
        float basePower;

        if (distanceRemaining > SLOWDOWN_DISTANCE_MM)
        {
            basePower = MAX_POWER;
        }
        else
        {
            float slowdownFrac = distanceRemaining / SLOWDOWN_DISTANCE_MM;
            basePower = MIN_POWER + (MAX_POWER - MIN_POWER) * slowdownFrac;
        }

        float headingError = normalizeAngle180(initialHeading - BrainInertial.heading(degrees));
        float correction = headingError * HEADING_P;
        MotorLeft.spin(forward, basePower * (1.0 + correction), percent);
        MotorRight.spin(forward, basePower * (1.0 - correction), percent);
        wait(10, msec);
    }

    DriveMotors.stop(hold);
    return true;
}

/*
    This function deletes the forward and reverse path
*/
bool deletePathByRoom(int start, int end, std::vector<Path> &pathLibrary, float (&adjacencyMatrix)[MAX_ROOMS + 1][MAX_ROOMS + 1])
{
    /*
      the assumption is that there is no reason to have a forward
      but not a reverse path since if there was an issue with one
      there should be the same issue with the other
    */
    int pathsDeleted = 0;
    bool forwardFound = false;
    for (int i = 0; i < pathLibrary.size() && !forwardFound; i++)
    {
        Path p = pathLibrary[i];
        if (p.startPoint == start && p.endPoint == end)
        {
            pathLibrary.erase(pathLibrary.begin() + i);
            createAdjacencyMatrix(pathLibrary, adjacencyMatrix);
            pathsDeleted++;
            forwardFound = true;
        }
    }
    bool reverseFound = false;
    for (int i = 0; i < pathLibrary.size() && !reverseFound; i++)
    {
        Path p = pathLibrary[i];

        if (p.startPoint == end && p.endPoint == start)
        {
            pathLibrary.erase(pathLibrary.begin() + i);
            createAdjacencyMatrix(pathLibrary, adjacencyMatrix);
            pathsDeleted++;
            reverseFound = true;
        }
    }

    if (pathsDeleted == 2)
        return true;
    else
        return false;
}

void relativeTurn(float angle_degrees)
{
    // uses a PI Controller
    const float TURN_TOLERANCE_DEG = 1.0;
    const float TURN_CONSTANT = 10.0;
    const float INTEG_CONSTANT = 0.01;

    // limit was set to prevent large
    // buildup on the integral term
    const float INTEG_MAX = 50.0;

    float errorSum = 0.0;
    BrainInertial.setRotation(0, degrees);

    float error = angle_degrees;

    while (std::fabs(error) > TURN_TOLERANCE_DEG)
    {

        float rotation_so_far = BrainInertial.rotation(degrees);
        error = angle_degrees - rotation_so_far;

        errorSum += error;

        // make sure the error sum stays within
        // the bound
        if (errorSum > INTEG_MAX)
        {
            errorSum = INTEG_MAX;
        }
        else if (errorSum < -INTEG_MAX)
        {
            errorSum = -1 * INTEG_MAX;
        }

        float motor_speed = TURN_CONSTANT * error + INTEG_CONSTANT * errorSum;

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
    wait(100, msec);
}

// ------------------------------------------
// CORE NAVIGATION LOGIC
// ------------------------------------------

bool runSinglePath(const Path &pathToRun)
{
    int pathLength = pathToRun.length;

    for (int i = 0; i < pathLength; i++)
    {
        // execute each step in each path sequentially
        wait(100, msec);
        int mode = pathToRun.steps[i][0];

        if (mode == DRIVE_MODE)
        {

            float distance_mm = pathToRun.steps[i][1];
            bool success = drive(distance_mm);
            if (!success)
                return false;
        }
        else
        { // TURN mode

            float target_heading = pathToRun.steps[i][1];
            float current_heading = BrainInertial.heading(degrees);
            // calculate the relative angle needed to turn
            float angleToTurn = normalizeAngle180(target_heading - current_heading);
            // by normalizing the angle we turn in the shorter direction
            relativeTurn(angleToTurn);
        }
    }
    return true;
}

bool executeJourney(const std::vector<int> &waypoints, const std::vector<Path> &pathLibrary)
{

    for (int i = 0; i < waypoints.size() - 1; i++)
    {
        // run the route between each pair
        // of waypoints
        int startPoint = waypoints[i];
        int endPoint = waypoints[i + 1];

        int pathIndice = findPath(startPoint, endPoint, pathLibrary);
        if (pathIndice != -1) //-1 corresponds to it not being found
        {
            bool success = runSinglePath(pathLibrary[pathIndice]);
            if (!success)
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    return true;
}
// ------------------------------------------
// TRAINING MODE - PATH RECORDING
// ------------------------------------------
void runTrainingMode(Path &pathToRecord)
{
    /*
    we are forcing turn drive turn drive etc etc as the sequence of movements
    this is to ensure the reverse path algorthim works as intended

    there are 3 big things happening here
    1. the control system to keep the robot from drifting to the side
    2. all the UI for displaying information about the route to the user
    3. data logging to store the information needed to recreate the route
    */
    const float INTEGRAL_LIMIT = 10;
    const float CORRECTION_LIMIT = 0.5;
    float headingErrorSum = 0.0; // Integral term for straightness
    // what the angle is trying to correct to
    float targetHeading = BrainInertial.heading(degrees);

    // set the starting values for the logging;
    pathToRecord.length = 0;
    pathToRecord.totalDistance_mm = 0;
    DriveMotors.resetPosition();
    float previous_encoder_turns = 0.0;

    // keep track of the states
    int currentState = TURN_MODE;
    bool finished = false;
    float lastEncoderTurns = DriveMotors.position(turns);

    // keep track of the UI
    int lastShownMode = -1;
    int lastShownLength = -1;
    float lastShownDisplacement = -1;

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("--- TRAINING ---");
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("R Up: LOG COMMIT");

    // While operators keeps adding steps
    while (!finished)
    {
        wait(100, msec);

        // we should only be updating the UI after a change is actually made
        // to prevent flickering

        // the following code is to display UI updates
        if (currentState != lastShownMode)
        {
            // update  the mode we are in

            Brain.Screen.setCursor(2, 1);

            if (currentState == DRIVE_MODE)
            {
                Brain.Screen.print("Mode: DRIVE");
            }
            else
            {
                Brain.Screen.print("Mode: DRIVE");
            }
            lastShownMode = currentState;
            Brain.Screen.setCursor(6, 1);

            // we only allow the user to exit when in turn mode
            if (currentState == TURN_MODE)
            {
                Brain.Screen.print("F Up: FINISH");
            }
        }

        // update the steps counts (length of the path)
        if (pathToRecord.length != lastShownLength)
        {
            Brain.Screen.setCursor(3, 1);
            Brain.Screen.print("Steps: %d   ", pathToRecord.length);
            lastShownLength = pathToRecord.length;
        }

        // now we show the displacement whether that be the angle turned or
        // distance travelled

        if (currentState == DRIVE_MODE)
        {
            float dist = turns_to_mm(DriveMotors.position(turns) - lastEncoderTurns);
            if (std::fabs(dist - lastShownDisplacement) > 10)
            // only update the ui if the change is greater than 1 cm
            {
                Brain.Screen.setCursor(4, 1);
                Brain.Screen.print("DRIVE: %.1f mm", dist);
                lastShownDisplacement = dist;
            }
        }
        else if (currentState == TURN_MODE)
        {
            float curHeading = BrainInertial.heading(degrees);
            if (std::fabs(curHeading - lastShownDisplacement) > 0.1)
            // only update the ui if the change is greater than 0.1 mm

            {
                Brain.Screen.setCursor(4, 1);
                Brain.Screen.print("TURN: %.1f deg", curHeading);
                lastShownDisplacement = curHeading;
            }
        }
        // ui update conditionals have ended

        // now we deal with the user logging a singular step in the path

        if (Controller.ButtonRUp.pressing() == 1)
        { // push in the segment

            DriveMotors.stop(hold);

            if (currentState == DRIVE_MODE)
            {
                // calculate the distance travelled in the drive segment
                float currentEncoderTurns = DriveMotors.position(turns);
                float changeTurns = currentEncoderTurns - lastEncoderTurns;
                float distance_mm = turns_to_mm(changeTurns);

                // add in those steps
                pathToRecord.steps[pathToRecord.length][0] = DRIVE_MODE;
                pathToRecord.steps[pathToRecord.length][1] = distance_mm;
                pathToRecord.totalDistance_mm += distance_mm;

                // reset base value
                lastEncoderTurns = currentEncoderTurns;
            }
            else
            { // turn mode
                float currentAbsoluteHeading = BrainInertial.heading(degrees);
                pathToRecord.steps[pathToRecord.length][0] = TURN_MODE;
                pathToRecord.steps[pathToRecord.length][1] = currentAbsoluteHeading;
            }
            pathToRecord.length++;

            // we need to swap the modes automatically once the user logs a step
            if (currentState == DRIVE_MODE)
            {
                currentState = TURN_MODE;
            }
            else
            {
                currentState = DRIVE_MODE;

                // store the target heading, this is what
                //  the PI controller aligns itself to
                targetHeading = BrainInertial.heading(degrees);

                // reset the eror sum for the PI Controller
                headingErrorSum = 0;
            }

            // force ui resets on these two values
            lastShownMode = 0;
            lastShownDisplacement = 0;

            // debounce button
            while (Controller.ButtonRUp.pressing())
            {
            }

            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(1, 1);
        }
        else if (Controller.ButtonFUp.pressing() && currentState == TURN_MODE)
        {
            // only allow them to end while in the tunr mode
            finished = true;
        }

        else
        { // default case where the user to move around
            if (currentState == DRIVE_MODE)
            {
                float drivePower = 0;

                if (Controller.AxisA.position() > 50)
                {
                    drivePower = 50;
                }
                else if (Controller.AxisA.position() < -50)
                {
                    drivePower = -50;
                }
                float sign = 0;
                if (drivePower > 0)
                {
                    sign = 1;
                }
                else
                {
                    sign = -1;
                }

                const float HEADING_CORRECTION_P = 0.03;

                const float HEADING_CORRECTION_I = 0.001;
                float deltaTurns = DriveMotors.position(turns) - previous_encoder_turns;
                float curHeading = BrainInertial.heading(degrees);
                float headingError = targetHeading - curHeading;

                headingError = normalizeAngle180(headingError);
                headingErrorSum += headingError * deltaTurns;

                // this keeps the integral within bounds to prevent large buildup
                // kept within the range [-INTEGRAL_LIMIT, INTEGRAL_LIMIT]
                headingErrorSum = std::min(INTEGRAL_LIMIT,
                                           std::max(-INTEGRAL_LIMIT, headingErrorSum));

                float correction = headingError * HEADING_CORRECTION_P + headingErrorSum * HEADING_CORRECTION_I;

                // this makes sure the correction is within a reasonable limit
                if (correction > CORRECTION_LIMIT)
                    correction = CORRECTION_LIMIT;
                if (correction < -CORRECTION_LIMIT)
                    correction = -CORRECTION_LIMIT;

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

    float finalHeading = BrainInertial.heading(degrees);
    /*
    manually add in the "turn" corresponding to the final
    robot heading
    this is needed for the reverse path algorithim
    */
    pathToRecord.steps[pathToRecord.length][0] = TURN_MODE;
    pathToRecord.steps[pathToRecord.length][1] = finalHeading;
    pathToRecord.length++;

    DriveMotors.stop(hold);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Path created!");
    wait(1000, msec);
}

// ------------------------------------------
// MAIN PROGRAM
// ------------------------------------------
int main()
{
    // Initializing Robot Configuration. DO NOT REMOVE!
    vexcodeInit();
    // Begin project code
    Brain.Screen.setFont(mono12);
    const int timeToOpen = 15;      // 15 Seconds till siren
    const float openPosition = 5.5; // Position for box to go to when opened
    const float lockPosition = 6.5;

    // Initializing Robot Configuration. DO NOT REMOVE!
    vexcodeInit();
    initRobot();

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    std::vector<Path> pathLibrary;

    // load in any paths stored in the file
    bool pathsFound = loadPathsFromFile(pathLibrary);

    wait(1000, msec);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);

    if (pathsFound)
    {
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Loaded %d paths! yay", pathLibrary.size());
    }
    else
    {
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("no saved data unfortunately");
    }

    wait(2000, msec);

    float adjacencyMatrix[MAX_ROOMS + 1][MAX_ROOMS + 1] = {};
    createAdjacencyMatrix(pathLibrary, adjacencyMatrix);

    bool programExit = false;
    while (!programExit)
    {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        std::string mode = mainMenu();

        /*
        possible modes: training (adding a route)
                      : autonomous (driving a route)
                      : deleting an existing route
                      : closing the robot
        */
        if (mode == "TRAINING")
        {
            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(1, 1);

            int start = getNumberInput("START ROOM:");
            int end = getNumberInput("ENDING ROOM:");

            if (findPath(start, end, pathLibrary) != -1)
            {
                Brain.Screen.clearScreen();
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.print("path exists");
            }
            // the case of start or end = -1 means they cancelled
            // so we can just send them back to the main menu
            else if (start != -1 && end != -1)
            {
                Path newPath;
                newPath.startPoint = start;
                newPath.endPoint = end;
                // allow the user to train the path between start and end
                runTrainingMode(newPath);

                // add both the forward AND backward path
                pathLibrary.push_back(newPath);
                pathLibrary.push_back(reversePath(newPath));
                createAdjacencyMatrix(pathLibrary, adjacencyMatrix);
            }
        }
        else if (mode == "AUTONOMOUS")
        {

            int start = getNumberInput("START ROOM:");
            int end = getNumberInput("ENDING ROOM:");
            if (start != -1 && end != -1)
            {
                // gets all the middle points that need to be taken
                // ex: to go from A -> D we need to do A -> B -> C -> D
                std::vector<int> waypoints = shortestPath(start, end, adjacencyMatrix);

                if (!waypoints.empty())
                {
                    Brain.Screen.clearScreen();
                    Brain.Screen.setCursor(1, 1);
                    Brain.Screen.print("Runing Route");
                    bool success = executeJourney(waypoints, pathLibrary);

                    if (success)
                    {
                        Brain.Screen.clearScreen();
                        Brain.Screen.setCursor(1, 1);
                        openBox(openPosition, lockPosition, timeToOpen);
                    }

                    else if (BumperSensor.pressing())
                    // error caused by bumper sensor
                    {
                        DriveMotors.stop(hold);
                        Brain.playSound(siren);
                        Brain.Screen.clearScreen();
                        Brain.Screen.setCursor(1, 1);
                        Brain.Screen.print("Emergency Stop! Ending Program");
                        wait(3000, msec);
                        programExit = true;
                    }
                    else // generic error message
                    {
                        Brain.Screen.clearScreen();
                        Brain.Screen.setCursor(1, 1);
                        Brain.Screen.print("MISSION FAILED / ABORTED");
                    }
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

            // both the forward and reverse paths should be deleted
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