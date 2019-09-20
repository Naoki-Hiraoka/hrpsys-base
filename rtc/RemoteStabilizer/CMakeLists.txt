option(BUILD_STABILIZER "Build Stabilizer RTC" ON)

if(NOT BUILD_STABILIZER)
  return()
endif()

set(comp_sources RemoteStabilizer.cpp RemoteStabilizerService_impl.cpp)
set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub)

add_library(RemoteStabilizer SHARED ${comp_sources})
target_link_libraries(RemoteStabilizer ${libs})
set_target_properties(RemoteStabilizer PROPERTIES PREFIX "")
add_executable(RemoteStabilizerComp RemoteStabilizerComp.cpp ${comp_sources})
target_link_libraries(RemoteStabilizerComp ${libs})

set(target RemoteStabilizer RemoteStabilizerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
