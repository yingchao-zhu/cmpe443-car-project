#ifndef PARAMETERS_H
#define PARAMETERS_H

#define ROBOT_SPEED 50
#define ROTATION_NUMBER_FOR_90_DEGREE 5

#define WIFI_SSID "HWLAB"
#define WIFI_PASSWORD "12345678"

#define OBSTACLE_DISTANCE 15
#define OBSTACLE_ESCAPE_DISTANCE 30
#define ULTRASONIC_MAX_DISTANCE 5000

#define START_MODE '*'

#define LIGHT_THRESHOLD 300
#define DIFFERENCE_THRESHOLD 50
#define TURN_THRESHOLD 1000

extern int TURN_LEFT_FLAG;
extern int TURN_RIGHT_FLAG;
extern int FORWARD_FLAG;
extern int BACKWARD_FLAG;
extern int IS_ESCAPING;

#endif
