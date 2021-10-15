/////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
// All rights reserved.
/////////////////////////////////////////////

#include "gaia_db.hpp"

#include "gaia/logger.hpp"

#include "zones.hpp"

using namespace gaia::danger_zone;

void initialize_zones()
{
    std::vector zones = {zones_t::c_green_zone, zones_t::c_yellow_zone, zones_t::c_red_zone};

    for (uint8_t zone_id : zones)
    {
        auto zone_iter = zone_t::list().where(zone_expr::id == zone_id);

        if (zone_iter.begin() == zone_iter.end())
        {
            gaia_log::app().info("Creating {} zone.", zones_t::zone_id_str(zone_id));
            zone_t::insert_row(zone_id);
        }
    }
}

object_t get_object(const char* object_id, const char* class_id)
{
    auto object_iter = object_t::list().where(
        object_expr::id == object_id);

    object_t object;

    if (object_iter.begin() == object_iter.end())
    {
        gaia_log::app().info("Found new object: {}", object_id);

        // The object_id is in the form: 'Person (12)'.
        object = object_t::get(
            object_t::insert_row(object_id, class_id, zones_t::c_no_zone));
    }
    else
    {
        object = *object_iter.begin();
    }

    return object;
}
