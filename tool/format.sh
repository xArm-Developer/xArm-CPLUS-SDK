#!/bin/bash

set -eu

clang-format-3.8 -style=Google -i xarm/*.cc
clang-format-3.8 -style=Google -i xarm/common/*.cc 
clang-format-3.8 -style=Google -i xarm/debug/*.cc 
clang-format-3.8 -style=Google -i xarm/instruction/*.cc 
clang-format-3.8 -style=Google -i xarm/linux/*.cc 
clang-format-3.8 -style=Google -i xarm/port/*.cc 
clang-format-3.8 -style=Google -i example/*.cc 

clang-format-3.8 -style=Google -i xarm/*.h
clang-format-3.8 -style=Google -i xarm/common/*.h
clang-format-3.8 -style=Google -i xarm/debug/*.h 
clang-format-3.8 -style=Google -i xarm/instruction/*.h 
clang-format-3.8 -style=Google -i xarm/linux/*.h
clang-format-3.8 -style=Google -i xarm/port/*.h 



echo " "
echo "------------------------"
echo "| format successfully! |"
echo "------------------------"
echo " "

