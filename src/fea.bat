@echo on
g++ -Wall -Wextra -Werror -std=c++17 -I"../Include" -o fea.exe fea.cpp

@echo off
rem Check if compilation was successful
if %errorlevel% neq 0 (
    echo Compilation failed.
    exit /b %errorlevel%
)
@echo Done.

@echo on
fea.exe