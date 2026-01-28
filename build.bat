@echo off
REM Quick build script
REM Run from the project root directory

if not exist build (
    echo Build directory not found. Running clean_build.bat first...
    call clean_build.bat
)

cd build
echo Building project...
make
echo.
echo Build complete! Executable: build\bin\MPC_DoublePendulum.exe
echo.
cd ..
