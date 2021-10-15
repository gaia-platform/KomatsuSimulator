/////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
// All rights reserved.
/////////////////////////////////////////////

#include "zones.hpp"

double zones_t::get_range(double x, double y)
{
    return std::sqrt(x * x + y * y);
}

uint8_t zones_t::get_range_zone_id(double x, double y)
{
    auto distance = get_range(x, y);

    for (auto range_id : c_range_id)
    {
        if (range_id[0] > distance)
        {
            return range_id[1];
        }
    }

    return c_no_zone;
}

double zones_t::get_direction(double z, double x)
{
    return std::atan2(z, x);
}

uint8_t zones_t::get_direction_zone_id(double z, double x)
{
    auto angle = get_direction(z, x);

    for (auto direction_id : c_direction_id)
    {
        if (angle < direction_id[0])
        {
            return direction_id[1];
        }
    }

    return c_default_angle_id;
}

uint8_t zones_t::convert_zone_id_to_simulation_id(uint8_t zone_id)
{
    if (zone_id == c_no_zone)
    {
        return 4;
    }

    return zone_id - 1;
}
