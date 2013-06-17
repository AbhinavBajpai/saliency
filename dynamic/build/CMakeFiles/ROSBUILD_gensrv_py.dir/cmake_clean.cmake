FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/dynamic/msg"
  "../src/dynamic/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/dynamic/srv/__init__.py"
  "../src/dynamic/srv/_Cap.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
