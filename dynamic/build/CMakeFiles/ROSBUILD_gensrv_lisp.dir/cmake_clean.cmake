FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/dynamic/msg"
  "../src/dynamic/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/Cap.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_Cap.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
