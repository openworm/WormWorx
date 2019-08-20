cmake_minimum_required(VERSION 3.4.1)

include_directories(sundials-2.3.0/include sundials-2.3.0/src)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

set(HEADER_FILES
            wormsim.h
            sundials-2.3.0/include/ida/ida.h
            sundials-2.3.0/include/ida/ida_band.h
            sundials-2.3.0/include/ida/ida_bbdpre.h
            sundials-2.3.0/include/ida/ida_dense.h
            sundials-2.3.0/include/ida/ida_spbcgs.h
            sundials-2.3.0/include/ida/ida_spgmr.h
            sundials-2.3.0/include/ida/ida_spils.h
            sundials-2.3.0/include/ida/ida_sptfqmr.h
            sundials-2.3.0/include/nvector/nvector_parallel.h
            sundials-2.3.0/include/nvector/nvector_serial.h
            sundials-2.3.0/include/sundials/sundials_band.h
            sundials-2.3.0/include/sundials/sundials_dense.h
            sundials-2.3.0/include/sundials/sundials_fnvector.h
            sundials-2.3.0/include/sundials/sundials_iterative.h
            sundials-2.3.0/include/sundials/sundials_math.h
            sundials-2.3.0/include/sundials/sundials_nvector.h
            sundials-2.3.0/include/sundials/sundials_smalldense.h
            sundials-2.3.0/include/sundials/sundials_spbcgs.h
            sundials-2.3.0/include/sundials/sundials_spgmr.h
            sundials-2.3.0/include/sundials/sundials_sptfqmr.h
            sundials-2.3.0/include/sundials/sundials_types.h
            )

add_library(wormworx_jni SHARED
            wormworx.cpp
            wormsim.cpp
            sundials-2.3.0/src/ida/ida.c
            sundials-2.3.0/src/ida/ida_band.c
            sundials-2.3.0/src/ida/ida_band_impl.h
            sundials-2.3.0/src/ida/ida_bbdpre.c
            sundials-2.3.0/src/ida/ida_bbdpre_impl.h
            sundials-2.3.0/src/ida/ida_dense.c
            sundials-2.3.0/src/ida/ida_dense_impl.h
            sundials-2.3.0/src/ida/ida_ic.c
            sundials-2.3.0/src/ida/ida_impl.h
            sundials-2.3.0/src/ida/ida_io.c
            sundials-2.3.0/src/ida/ida_spbcgs.c
            sundials-2.3.0/src/ida/ida_spgmr.c
            sundials-2.3.0/src/ida/ida_spils.c
            sundials-2.3.0/src/ida/ida_spils_impl.h
            sundials-2.3.0/src/ida/ida_sptfqmr.c
            sundials-2.3.0/src/nvec_ser/fnvector_serial.c
            sundials-2.3.0/src/nvec_ser/fnvector_serial.h
            sundials-2.3.0/src/nvec_ser/nvector_serial.c
            sundials-2.3.0/src/sundials/sundials_band.c
            sundials-2.3.0/src/sundials/sundials_config.h
            sundials-2.3.0/src/sundials/sundials_config.in
            sundials-2.3.0/src/sundials/sundials_dense.c
            sundials-2.3.0/src/sundials/sundials_iterative.c
            sundials-2.3.0/src/sundials/sundials_math.c
            sundials-2.3.0/src/sundials/sundials_nvector.c
            sundials-2.3.0/src/sundials/sundials_smalldense.c
            sundials-2.3.0/src/sundials/sundials_spbcgs.c
            sundials-2.3.0/src/sundials/sundials_spgmr.c
            sundials-2.3.0/src/sundials/sundials_sptfqmr.c
            )

# add lib dependencies
target_link_libraries(wormworx_jni
                      android
                      log 
                      EGL
                      GLESv2)

