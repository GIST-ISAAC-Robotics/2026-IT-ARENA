cmake_minimum_required(VERSION 3.22)
# 검사 전용 SDK 프로젝트. 모터/서보/센서/통신 어댑터가 아니다.
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(arena_actuation_smoke)
