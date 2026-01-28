@echo off
REM Clean build script for MPC_DoublePendulum
REM Removes all generated CMake files and rebuilds from scratch

echo Cleaning build directory...
rmdir /s /q build
echo.

echo Creating fresh build directory...
mkdir build
cd build
echo.

echo Running CMake configuration...
cmake -G "MinGW Makefiles" ..
echo.

echo Build setup complete!
echo Running make...
make
echo.
echo Clean build complete! Executable: build\bin\MPC_DoublePendulum.exe
echo.
cd ..