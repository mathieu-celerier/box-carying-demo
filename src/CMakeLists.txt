set(controller_SRC BoxDemoController.cpp BoxDemoController_WorkspaceConstraint.cpp)

set(controller_HDR BoxDemoController.h BoxDemoController_WorkspaceConstraint.h)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS
                                                 "-DBoxDemoController_EXPORTS")
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm)
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION ${MC_RTC_LIBDIR}
  EXPORT ${TARGETS_EXPORT_NAME})

add_controller(${PROJECT_NAME}_controller lib.cpp "")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

add_subdirectory(states)
