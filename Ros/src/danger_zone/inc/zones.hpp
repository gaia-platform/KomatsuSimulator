#pragma once

//*****************************************************************************
//*
//* Dependencies: Requires Gaia preview (without libc++) or later, Gaia March
//* 2021 will not work
//*
//*****************************************************************************

#include <cmath>

#include <memory>

class zones_t
{
private:
    static inline std::shared_ptr<zones_t> m_singleton;

    const double rad_per_deg = 0.0174533; //(pi / 180)
    const double red_zone = 10.0;
    const double yellow_zone = 20.0;
    const double green_zone = 2000.0;

    double m_range_id[3][2] = {
        {red_zone, 0},
        {yellow_zone, 1},
        {green_zone, 2}};

    double m_default_range_id = 4;

    double m_direction_id[6][2] = {
        {60 * rad_per_deg, 0},
        {120 * rad_per_deg, 1},
        {180 * rad_per_deg, 2},
        {240 * rad_per_deg, 3},
        {300 * rad_per_deg, 4},
        {360 * rad_per_deg, 5}};

    double m_default_angle_id = 6;

public:
    //*****************************************************************************
    //*
    //* args:
    //*   double x : one axis
    //*   double y : the other axis
    //*
    //*****************************************************************************

    static double get_range(double x, double y)
    {
        return std::sqrt(x * x + y * y);
    }

    //*****************************************************************************
    //*
    //* args:
    //*   double x : one axis
    //*   double y : the other axis
    //*
    //*****************************************************************************

    u_int get_range_zone_id(double x, double y)
    {
        auto distance = get_range(x, y);

        for (auto rid : m_range_id)
        {
            if (distance < rid[0])
            {
                return rid[1];
            }
        }

        return m_default_range_id;
    }

    //*****************************************************************************
    //*
    //* args:
    //*   double z : front +
    //*   double x : right +
    //*
    //*****************************************************************************

    static double get_direction(double z, double x)
    {
        return std::atan2(z, x);
    }

    //*****************************************************************************
    //*
    //* args:
    //*   double z : front +
    //*   double x : right +
    //*
    //*****************************************************************************

    u_int get_direction_zone_id(double z, double x)
    {
        auto angle = get_direction(z, x);

        for (auto rid : m_direction_id)
        {
            if (angle < rid[0])
            {
                return rid[1];
            }
        }

        return m_default_angle_id;
    }

    static std::shared_ptr<zones_t> get_singleton()
    {
        if (nullptr == m_singleton)
        {
            m_singleton = std::make_shared<zones_t>();
        }

        return m_singleton;
    }
};
