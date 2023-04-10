#ifndef DATA_ENUMS_HPP
#define DATA_ENUMS_HPP

enum CarState
{
    CAR_IN_CURVE,
    CAR_IN_LINE,
    CAR_STOPPED,
    CAR_MAPPING,
};

enum CarSensor
{
    CAR_SENSOR_LEFT,
    CAR_SENSOR_RIGHT,
    CAR_SENSOR_FRONT,
};

enum TrackState
{
    LINE,
    CURVE,
    ZIGZAG,
    MAPPING,
};

#endif