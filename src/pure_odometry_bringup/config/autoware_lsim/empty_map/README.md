# Empty map placeholder

`autoware.launch.xml` requires a `map_path` argument even when `launch_map:=false`.
The localization-only LSim launch points at this directory by default and never starts
Autoware's map component. No PCD or Lanelet2 map is loaded from this directory.
