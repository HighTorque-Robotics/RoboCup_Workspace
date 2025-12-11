cmake_minimum_required(VERSION 2.8.3)
project(dmsgs)

find_package(catkin REQUIRED geometry_msgs message_generation)

add_message_files(
  FILES
  # dbehavior
  BehaviorInfo.msg

  # dmotion
  HeadCommand.msg
  BodyCommand.msg
  ActionCommand.msg
  MotionInfo.msg
  FieldFeature.msg
  IMUDebug.msg
  MotionDebugInfo.msg
  FeetPressure.msg
  
  # dvision
  ParticleMsg.msg
  SaveImg.msg
  Line.msg
  VisionInfo.msg

  # dnetwork
  GCInfo.msg
  TeamInfo.msg

  # dserver
  ServerInfo.msg
  MotorState.msg

  Joy.msg
)

add_service_files(
    FILES
    ToggleAMCL.srv
    ResetParticleTouchLine.srv
    ResetParticleLeftTouch.srv
    ResetParticleRightTouch.srv
    ResetParticlePoint.srv
    ResetParticleRandom.srv
    SetInitOrientation.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)

catkin_package(
    #  INCLUDE_DIRS include
#  LIBRARIES dmsgs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

## BUG, without building this, the dependency will crash
add_executable(dmsgs_foo main.cpp)
add_dependencies(dmsgs_foo ${PROJECT_NAME}_generate_messages_cpp)
