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
