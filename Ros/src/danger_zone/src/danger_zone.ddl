create table if not exists detectron (
      id string active
);

create table if not exists detected (
      count int32 active,
      changed int32 active
);

create table if not exists dobject (
      id int32,
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
      dobject__detected references detected
);



