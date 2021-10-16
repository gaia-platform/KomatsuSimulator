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

void dump_zone(const zone_t& zone)
{
    printf("zone id:                %d\n", zone.id());
}

void dump_object(const object_t& object)
{
    printf("object id:              %s\n", object.id());
    printf("object class id:        %s\n", object.class_id());
    printf("object zone id:         %d\n", object.zone_id());
}

void dump_detection(const detection_t& detection)
{
    printf("detection frame id:     %s\n", detection.frame_id());
    printf("detection seconds:      %d\n", detection.seconds());
    printf("detection nseconds:     %d\n", detection.nseconds());
}

void dump_d_object(const d_object_t& d_object)
{
    printf("d_object id:            %s\n", d_object.object_id());
    printf("d_object score:         %f\n", d_object.score());
    printf("d_object range:         %d\n", d_object.range_id());
    printf("d_object direction:     %d\n", d_object.direction_id());
    printf("d_object pos:           x = %f, y = %f, z = %f\n",
        d_object.pos_x(), d_object.pos_y(), d_object.pos_z());
    printf("d_object size:          x = %f, y = %f, z = %f\n",
        d_object.size_x(), d_object.size_y(), d_object.size_z());
    printf("d_object orient:        x = %f, y = %f, z = %f, w = %f\n",
        d_object.orient_x(), d_object.orient_y(), d_object.orient_z(), d_object.orient_w());
    printf("d_object zone id:       %d\n", d_object.zone_id());
}

void dump_zone_transition_event(const zone_transition_event_t& zone_transition_event)
{
    printf("zone transition event object id:    %s\n", zone_transition_event.object_id());
    printf("zone transition event from zone id: %d\n", zone_transition_event.from_zone_id());
    printf("zone transition event to zone id:   %d\n", zone_transition_event.to_zone_id());
}

void dump_zones()
{
    printf("--------------------------------------------------------\n");
    printf("ZONES---------------------------------------------------\n");
    for (const auto& zone : zone_t::list())
    {
        printf("--------------------------------------------------------\n");
        dump_zone(zone);
    }
    printf("--------------------------------------------------------\n");
}

void dump_objects()
{
    printf("--------------------------------------------------------\n");
    printf("OBJECTS-------------------------------------------------\n");
    for (const auto& object : object_t::list())
    {
        printf("--------------------------------------------------------\n");
        dump_object(object);
    }
    printf("--------------------------------------------------------\n");
}

void dump_detections()
{
    printf("--------------------------------------------------------\n");
    printf("DETECTIONS----------------------------------------------\n");
    for (const auto& detection : detection_t::list())
    {
        printf("--------------------------------------------------------\n");
        dump_detection(detection);
    }
    printf("--------------------------------------------------------\n");
}

void dump_d_objects()
{
    printf("--------------------------------------------------------\n");
    printf("D_OBJECTS-----------------------------------------------\n");
    for (const auto& d_object : d_object_t::list())
    {
        printf("--------------------------------------------------------\n");
        dump_d_object(d_object);
    }
    printf("--------------------------------------------------------\n");
}

void dump_zone_transition_events()
{
    printf("--------------------------------------------------------\n");
    printf("ZONE_TRANSITIION_EVENTS---------------------------------\n");
    for (const auto& zone_transition_event : zone_transition_event_t::list())
    {
        printf("--------------------------------------------------------\n");
        dump_zone_transition_event(zone_transition_event);
    }
    printf("--------------------------------------------------------\n");
}

void dump_db_state()
{
    printf("\n");
    dump_objects();
    dump_detections();
    dump_d_objects();
    dump_zone_transition_events();
}

void dump_db()
{
    printf("\n");
    dump_zones();
    dump_db_state();
}
