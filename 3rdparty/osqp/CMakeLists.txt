if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/osqp)
  include(ExternalProject)
  ExternalProject_Add(osqpproj
  GIT_REPOSITORY https://github.com/oxfordcontrol/osqp.git
  GIT_TAG v0.5.0
  TIMEOUT 30
  CMAKE_ARGS -DDFLOAT=ON
  BUILD_COMMAND make -j
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND sh -c "mkdir ${CMAKE_CURRENT_BINARY_DIR}/include -p" && sh -c "cp -r include/* ${CMAKE_CURRENT_BINARY_DIR}/include/" && sh -c "rm -fr ${CMAKE_CURRENT_BINARY_DIR}/include/osqp/.gitignore" && sh -c "mkdir ${CMAKE_CURRENT_BINARY_DIR}/lib -p" && cp out/libosqp.so ${CMAKE_CURRENT_BINARY_DIR}/lib/
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  )
endif()
