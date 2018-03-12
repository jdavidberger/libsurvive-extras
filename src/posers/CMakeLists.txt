list(APPEND srcs
  init_extras.cc init_extras.h
  opencv_solver.cc opencv_solver.h
  sba_solver.cc sba_solver.h
        PersistentScene.cc PersistentScene.h
  )

add_library(solvers ${srcs})

target_link_libraries(solvers debug ${SURVIVE_DEBUG_LIBRARY} )
target_link_libraries(solvers optimized ${SURVIVE_LIBRARY})
target_link_libraries(solvers ${SBA_LIBRARY} lapack ${OpenCV_LIBS})

add_executable(configuration_solver main.cc)
target_link_libraries(configuration_solver solvers)
