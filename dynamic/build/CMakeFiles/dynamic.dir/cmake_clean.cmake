FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/dynamic/msg"
  "../src/dynamic/srv"
  "../msg_gen"
  "../srv_gen"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/dynamic.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
