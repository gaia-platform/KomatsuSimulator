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
      pos_x float,
      pos_y float,      
      size_x float,
      size_y float,
      dobject__detected references detected
);



