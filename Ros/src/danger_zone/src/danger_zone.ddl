database danger_zone

table detectron (
    id string unique
)

table detected (
    count int32,
    changed int32,
    dobjects references dobject[]
)

table object (
    object_id string,
    class_id string
)

table dobject (
    -- This id arrives from the simulation and it is a non-unique string.
    -- It most likely denotes the ID of an object within the simulation
    -- and it can't be used as ID of an "observer object" because the same object
    -- can be observed many times.
    object_id string,
    class_id string,
    score float,
    frame_id string,
    range_id int32,
    direction_id int32,
    seconds int32,
    nseconds int32,
    pos_x float,
    pos_y float,
    pos_z float,
    size_x float,
    size_y float,
    size_z float,
    orient_x float,
    orient_y float,
    orient_z float,
    orient_w float,
    detected references detected
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
