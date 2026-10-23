cmake_minimum_required(VERSION 3.22)
# 출력 비활성 UART bring-up 검사 프로젝트. 모터/서보/센서 구동 펌웨어가 아니다.
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(arena_actuation_uart_bench)
