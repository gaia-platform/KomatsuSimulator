database danger_zone

table detectron (
    id string unique
)

table detected (
    count int32,
    changed int32,
    dobjects references dobject[]
)

table dobject (
    id int32 unique,
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

-- We need representation of zones & machines and criteria that would trigger the data logging.
-- Question? how to structure the database to represent more complicated scenarios.
-- Question? Is there a difference between the different types of objects seen?
-- Josh's buffer. I can tell to that node "catch a snapshot now" and he will write to the file system.
    -- Understand the boundaries between ROS & Gaia in regards to storing/manipulating data.
    -- The message we are going to send: start logging at this time and include this metadata.
-- Save data only on transition between zones. We have a new table that tracks objects of interest and we log objects
--   that transition between zones. This can trigger a message that save the telemetry.
-- Organization for dobjects, we wanna keep them and not dispose them?
