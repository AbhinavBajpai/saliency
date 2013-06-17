FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/dynamic/msg"
  "../src/dynamic/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/dynamic/msg/__init__.py"
  "../src/dynamic/msg/_Cap.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
