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

table object (
    -- For now this ID is just a merge of class_id + object_id. Will change.
    id string unique,
    object_id string,
    class_id string
)

table d_object (
    -- Does not create a real relationship to the object to reduce contention on insertion/deletion.
    object_id string,

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

    -- Zone, numeric values gets 0 as default value,
    -- which corresponds to zones_t::c_no_zone
    zone uint8,

    detection references detection
)



--
-- Actions
--
-- Gaia rules are executed within a database transaction. If the transaction fail,
-- the rules engine will retry it as many times as specified in the gaia_log.conf
-- (3 times by default). For this reason rules need to be idempotent: their effect
-- on the database must be the same given the same initial database state.
--
-- Idempotency is broken as soon as non-transactional code is mixed with transactional
-- code. Sending ROS messages is an example of non-transactional code. For this reason
-- it is ideal to separate code that deal with the database from the code that deals with
-- external world. You can do this by creating rules which only purpose is to deal with
-- the external world and that are triggered by specific "action" records.

table send_obstacle_array_msg_action (

)


table send_trigger_log_action (

)
