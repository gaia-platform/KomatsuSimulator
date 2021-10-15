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
