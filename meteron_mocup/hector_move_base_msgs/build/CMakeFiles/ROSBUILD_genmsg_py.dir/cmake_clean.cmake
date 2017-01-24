FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/hector_move_base_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/hector_move_base_msgs/msg/__init__.py"
  "../src/hector_move_base_msgs/msg/_MoveBasePath.py"
  "../src/hector_move_base_msgs/msg/_MoveBaseActionResult.py"
  "../src/hector_move_base_msgs/msg/_MoveBaseFeedback.py"
  "../src/hector_move_base_msgs/msg/_MoveBaseActionGoal.py"
  "../src/hector_move_base_msgs/msg/_MoveBaseAction.py"
  "../src/hector_move_base_msgs/msg/_MoveBaseActionFeedback.py"
  "../src/hector_move_base_msgs/msg/_MoveBaseActionExplore.py"
  "../src/hector_move_base_msgs/msg/_MoveBaseResult.py"
  "../src/hector_move_base_msgs/msg/_MoveBaseGoal.py"
  "../src/hector_move_base_msgs/msg/_MoveBaseActionPath.py"
  "../src/hector_move_base_msgs/msg/_MoveBaseExplore.py"
  "../src/hector_move_base_msgs/msg/_MoveBaseActionGeneric.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
