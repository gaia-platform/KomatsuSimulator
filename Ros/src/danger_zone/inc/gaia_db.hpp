/////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
// All rights reserved.
/////////////////////////////////////////////

#pragma once

#include "gaia_danger_zone.h"

// Initialize zones table.
void initialize_zones();

// Retrieve an object for a given object id and class id.
// If the object does not exist, create it.
gaia::danger_zone::object_t get_object(const char* object_id, const char* class_id);

// Dump the content of specific tables.
void dump_zones();
void dump_objects();
void dump_detections();
void dump_d_objects();
void dump_zone_transition_events();

// Dump the content of all tables that are expected to change.
void dump_db_state();

// Dump the entire database, including tables whose content is not expected to change.
void dump_db();
