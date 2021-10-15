/////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
// All rights reserved.
/////////////////////////////////////////////

#pragma once

#include <cmath>

#include <memory>

class zones_t
{
public:
    static constexpr uint8_t c_no_zone = 0;
    static constexpr uint8_t c_red_zone = 1;
    static constexpr uint8_t c_yellow_zone = 2;
    static constexpr uint8_t c_green_zone = 3;

public:
    static uint8_t get_range_zone_id(double x, double y);

    static uint8_t get_direction_zone_id(double z, double x);

private:
    static constexpr double c_rad_per_deg = 0.0174533; //(pi / 180)
    static constexpr double c_red_zone_radius = 10.0;
    static constexpr double c_yellow_zone_radius = 20.0;
    static constexpr double c_green_zone_radius = 2000.0;

    static constexpr double c_range_id[3][2] = {
        // The order is important.
        {c_red_zone_radius, c_red_zone},
        {c_yellow_zone_radius, c_yellow_zone},
        {c_green_zone_radius, c_green_zone}};

    static constexpr double c_direction_id[6][2] = {
        {60 * c_rad_per_deg, 0},
        {120 * c_rad_per_deg, 1},
        {180 * c_rad_per_deg, 2},
        {240 * c_rad_per_deg, 3},
        {300 * c_rad_per_deg, 4},
        {360 * c_rad_per_deg, 5}};

    static constexpr double c_default_angle_id = 6;

private:
    /**
     * ind the distance from the sensor to the detected object.
     */
    static double get_range(double x, double y);

    /**
     * Find the direction of the object relative to the sensor.
     */
    static double get_direction(double z, double x);
};
