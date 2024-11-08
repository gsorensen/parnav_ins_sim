cmake_minimum_required(VERSION 3.15)
project(gtsam-conan-project CXX)

set(CMAKE_CXX_STANDARD 23)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -D_GLIBCXX_DEBUG -DMYDEBUGOUTPUT")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(GTSAM REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

function(link_common name)
    target_link_libraries(${name} Eigen3::Eigen)
    target_link_libraries(${name} gtsam::gtsam)
    target_link_libraries(${name} fmt::fmt)
endfunction()

function(link_def name val)
    add_compile_definitions(${name} PRIVATE ${val})
endfunction()

message(${GTSAM_INCLUDE_DIR})
message(${Eigen3_INCLUDE_DIR})

include_directories(${GTSAM_INCLUDE_DIR})
include_directories(${Eigen3_INCLUDE_DIR})

set(SOURCE_FILES 
    src/RawSimulationData.cpp
    src/IMU.cpp
    src/GNSS.cpp
    src/PARS.cpp
    src/OutlierRejection.cpp
    src/PARSRangeFactor.cpp
    src/PARSAzimuthFactor.cpp
    src/PARSElevationFactor.cpp
    src/FactorGraphOptimiser.cpp
    src/utils.cpp
    )

add_library(FGO_COMMON OBJECT ${SOURCE_FILES})
target_compile_definitions(FGO_COMMON PUBLIC TRANS_POS)
target_compile_definitions(FGO_COMMON PUBLIC ROT_POS)
link_common(FGO_COMMON)

set(FIXED_LAG_RANGE_ONLY fixed_lag_range_only)
add_executable(${FIXED_LAG_RANGE_ONLY} src/main.cpp)
target_compile_definitions(${FIXED_LAG_RANGE_ONLY} PRIVATE FIXED_LAG)
target_compile_definitions(${FIXED_LAG_RANGE_ONLY} PRIVATE RANGE_ONLY)
target_compile_definitions(${FIXED_LAG_RANGE_ONLY} PRIVATE OTTER)
target_link_libraries(${FIXED_LAG_RANGE_ONLY} FGO_COMMON)
#link_common(${FIXED_LAG_RANGE_ONLY})

set(FIXED_LAG_FULL_PARS fixed_lag_full_pars)
add_executable(${FIXED_LAG_FULL_PARS} src/main.cpp)
target_compile_definitions(${FIXED_LAG_FULL_PARS} PRIVATE FIXED_LAG)
target_compile_definitions(${FIXED_LAG_FULL_PARS} PRIVATE FULL_PARS)
target_compile_definitions(${FIXED_LAG_FULL_PARS} PRIVATE OTTER)
target_link_libraries(${FIXED_LAG_FULL_PARS} FGO_COMMON)
#link_common(${FIXED_LAG_FULL_PARS})
set(FIXED_LAG_FULL_PARS_SENSOR_FAULT fixed_lag_full_pars_sensor_fault)
add_executable(${FIXED_LAG_FULL_PARS_SENSOR_FAULT} src/main.cpp)
target_compile_definitions(${FIXED_LAG_FULL_PARS_SENSOR_FAULT} PRIVATE FIXED_LAG)
target_compile_definitions(${FIXED_LAG_FULL_PARS_SENSOR_FAULT} PRIVATE FULL_PARS)
target_compile_definitions(${FIXED_LAG_FULL_PARS_SENSOR_FAULT} PRIVATE OTTER)
target_compile_definitions(${FIXED_LAG_FULL_PARS_SENSOR_FAULT} PRIVATE SIMULATE_FAULTS)
target_link_libraries(${FIXED_LAG_FULL_PARS_SENSOR_FAULT} FGO_COMMON)

set(FIXED_LAG_FULL_PARS_OUTLIER_REJECTION fixed_lag_full_pars_outlier_rejection)
add_executable(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION} src/main.cpp)
target_compile_definitions(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION} PRIVATE FIXED_LAG)
target_compile_definitions(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION} PRIVATE FULL_PARS)
target_compile_definitions(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION} PRIVATE OTTER)
target_compile_definitions(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION} PRIVATE SIMULATE_FAULTS)
target_compile_definitions(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION} PRIVATE OUTLIER_REJECTION)
target_compile_definitions(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION} PRIVATE USE_NATURAL_TEST)
target_link_libraries(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION} FGO_COMMON)

set(FIXED_LAG_FULL_PARS_OUTLIER_REJECTION_HUBER fixed_lag_full_pars_outlier_rejection_huber)
add_executable(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION_HUBER} src/main.cpp)
target_compile_definitions(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION_HUBER} PRIVATE FIXED_LAG)
target_compile_definitions(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION_HUBER} PRIVATE FULL_PARS)
target_compile_definitions(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION_HUBER} PRIVATE OTTER)
target_compile_definitions(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION_HUBER} PRIVATE SIMULATE_FAULTS)
target_compile_definitions(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION_HUBER} PRIVATE OUTLIER_REJECTION)
target_compile_definitions(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION_HUBER} PRIVATE USE_HUBER)
target_link_libraries(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION_HUBER} FGO_COMMON)

set(FIXED_LAG_FULL_PARS_OUTLIER_REJECTION_TUKEY fixed_lag_full_pars_outlier_rejection_tukey)
add_executable(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION_TUKEY} src/main.cpp)
target_compile_definitions(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION_TUKEY} PRIVATE FIXED_LAG)
target_compile_definitions(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION_TUKEY} PRIVATE FULL_PARS)
target_compile_definitions(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION_TUKEY} PRIVATE OTTER)
target_compile_definitions(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION_TUKEY} PRIVATE SIMULATE_FAULTS)
target_compile_definitions(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION_TUKEY} PRIVATE OUTLIER_REJECTION)
target_compile_definitions(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION_TUKEY} PRIVATE USE_TUKEY)
target_link_libraries(${FIXED_LAG_FULL_PARS_OUTLIER_REJECTION_TUKEY} FGO_COMMON)

set(DUMMY_FIXED_LAG_RANGE_ONLY dummy_fixed_lag_range_only)
add_executable(${DUMMY_FIXED_LAG_RANGE_ONLY} src/main.cpp)
target_compile_definitions(${DUMMY_FIXED_LAG_RANGE_ONLY} PRIVATE FIXED_LAG)
target_compile_definitions(${DUMMY_FIXED_LAG_RANGE_ONLY} PRIVATE RANGE_ONLY)
target_compile_definitions(${DUMMY_FIXED_LAG_RANGE_ONLY} PRIVATE DUMMY)
target_link_libraries(${DUMMY_FIXED_LAG_RANGE_ONLY} FGO_COMMON)
#link_common(${DUMMY_FIXED_LAG_RANGE_ONLY})
#
set(DUMMY_FIXED_LAG_FULL_PARS dummy_fixed_lag_full_pars)
add_executable(${DUMMY_FIXED_LAG_FULL_PARS} src/main.cpp)
target_compile_definitions(${DUMMY_FIXED_LAG_FULL_PARS} PRIVATE FIXED_LAG)
target_compile_definitions(${DUMMY_FIXED_LAG_FULL_PARS} PRIVATE FULL_PARS)
target_compile_definitions(${DUMMY_FIXED_LAG_FULL_PARS} PRIVATE DUMMY)
target_link_libraries(${DUMMY_FIXED_LAG_FULL_PARS} FGO_COMMON)
#link_common(${DUMMY_FIXED_LAG_FULL_PARS})

set(FIXED_LAG_GNSS fixed_lag_gnss)
add_executable(${FIXED_LAG_GNSS} src/main.cpp)
target_compile_definitions(${FIXED_LAG_GNSS} PRIVATE FIXED_LAG)
target_compile_definitions(${FIXED_LAG_GNSS} PRIVATE GNSS_BENCHMARK)
target_compile_definitions(${FIXED_LAG_GNSS} PRIVATE OTTER)
target_link_libraries(${FIXED_LAG_GNSS} FGO_COMMON)

set(DUMMY_FIXED_LAG_GNSS dummy_fixed_lag_gnss)
add_executable(${DUMMY_FIXED_LAG_GNSS} src/main.cpp)
target_compile_definitions(${DUMMY_FIXED_LAG_GNSS} PRIVATE FIXED_LAG)
target_compile_definitions(${DUMMY_FIXED_LAG_GNSS} PRIVATE GNSS_BENCHMARK)
target_compile_definitions(${DUMMY_FIXED_LAG_GNSS} PRIVATE DUMMY)
target_link_libraries(${DUMMY_FIXED_LAG_GNSS} FGO_COMMON)

set(STANDSTILL_FIXED_LAG_GNSS standstill_fixed_lag_gnss)
add_executable(${STANDSTILL_FIXED_LAG_GNSS} src/main.cpp)
target_compile_definitions(${STANDSTILL_FIXED_LAG_GNSS} PRIVATE FIXED_LAG)
target_compile_definitions(${STANDSTILL_FIXED_LAG_GNSS} PRIVATE GNSS_BENCHMARK)
target_link_libraries(${STANDSTILL_FIXED_LAG_GNSS} FGO_COMMON)
