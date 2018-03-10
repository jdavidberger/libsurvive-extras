list(APPEND srcs
  init_extras.cc init_extras.h
  opencv_solver.cc opencv_solver.h
  sba_solver.cc sba_solver.h
  survive_reproject.c survive_reproject.h
  )

add_library(solvers ${srcs})
target_link_libraries(solvers ${SBA_LIBRARY} lapack ${SURVIVE_LIBRARY} ${OpenCV_LIBS})

add_executable(configuration_solver main.cc)
target_link_libraries(configuration_solver solvers)
