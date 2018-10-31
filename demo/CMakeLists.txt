# 3.10 introduces C++ 17 support
cmake_minimum_required (VERSION 3.10)
project (mpt_demos)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED)
#find_package(FCL REQUIRED)
find_package(Assimp REQUIRED)

######### Find CCD the hard way
find_package(PkgConfig)
if (PKGCONFIG_FOUND)
    pkg_check_modules(FCL fcl>=0.6)
    pkg_check_modules(CCD ccd>=2.0)
else()
    find_package(FCL REQUIRED)
    find_package(CCD CONFIG)
endif()
#########

link_directories(${FCL_LIBRARY_DIRS} ${ASSIMP_LIBRARY_DIRS} ${CCD_LIBRARY_DIRS})
include_directories(../src ../../nigh/src ${FCL_INCLUDE_DIRS} ${CCD_LIBRARY_DIRS} ${ASSIMP_INCLUDE_DIRS})

if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

foreach (nn KDTreeBatch GNAT)
    foreach (scalar float double)
        foreach (mt 0 1)
            if (mt)
                set(mtname mt)
            else()
                set(mtname st)
            endif()
            
            set(trgt nao_cup_planning-${nn}-${scalar}-${mtname})
            add_executable(${trgt} nao_cup_planning.cpp)
            target_compile_definitions(${trgt} PRIVATE NN_TYPE=${nn} SCALAR_TYPE=${scalar} MT=1)
            target_link_libraries(${trgt} Eigen3::Eigen)

            set(trgt se3_rigid_body_planning-${nn}-${scalar}-${mtname})
            add_executable(${trgt} se3_rigid_body_planning.cpp)
            target_compile_definitions(${trgt} PRIVATE NN_TYPE=${nn} SCALAR_TYPE=${scalar} MT=1)
            target_link_libraries(${trgt} Eigen3::Eigen ${ASSIMP_LIBRARIES} ${FCL_LIBRARIES} ${CCD_LIBRARIES})

	endforeach(mt)
    endforeach(scalar)
endforeach(nn)

set(trgt holonomic_2d_point_planning)
add_executable(${trgt} holonomic_2d_point_planning.cpp)
target_link_libraries(${trgt} Eigen3::Eigen)

