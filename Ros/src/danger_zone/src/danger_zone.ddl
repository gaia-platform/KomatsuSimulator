create database if not exists danger_zone;

use danger_zone;

create table if not exists detectron (
      detid string active
);

create table if not exists detected (
      count int32 active,
      changed int32 active
);

create table if not exists dobject (
      obid int32,
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
      orient_w float
);

create relationship if not exists detected_dobjects (
    detected.dobjects -> dobject[],
    dobject.detected -> detected
);



