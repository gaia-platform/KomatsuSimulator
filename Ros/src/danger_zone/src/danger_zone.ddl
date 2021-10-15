---------------------------------------------
-- Copyright (c) Gaia Platform LLC
-- All rights reserved.
---------------------------------------------

database danger_zone

table detection (
    d_objects references d_object[],

    -- Identifier of detection frame.
    frame_id string,

    -- Seconds/nanoseconds of detection frame.
    seconds int32,
    nseconds int32
)

table d_object (
    -- This id arrives from the simulation and it is a non-unique string.
    -- It most likely denotes the ID of an object within the simulation
    -- and it can't be used as ID of an "observer object" because the same object
    -- can be observed many times.
    object_id string,

    -- Class of detected object.
    class_id string,

    -- Detection score.
    score float,

    -- Identifier of detection frame.
    frame_id string,

    -- Range and direction detected.
    range_id int32,
    direction_id int32,

    -- Seconds/nanoseconds of detection frame.
    seconds int32,
    nseconds int32,

    -- Position coordinates.
    pos_x float,
    pos_y float,
    pos_z float,

    -- Box dimensions.
    size_x float,
    size_y float,
    size_z float,

    -- Pose coordinates.
    orient_x float,
    orient_y float,
    orient_z float,
    orient_w float,

    detection references detection
)
