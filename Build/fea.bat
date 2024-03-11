@echo on
g++ -Wall -Wextra -Werror -std=c++11 -o fea.exe fea.cpp

@echo off
rem Check if compilation was successful
if %errorlevel% neq 0 (
    echo Compilation failed.
    exit /b %errorlevel%
)

@echo on
fea.exe

@echo.
@echo Done.