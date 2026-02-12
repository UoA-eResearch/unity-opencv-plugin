@echo off
echo ==========================================
echo Building OpenCvPlugin for Unity (x64)
echo ==========================================

:: 1. Create Build Directory
if not exist build mkdir build
cd build

:: 2. Configure the project (wrangle dependencies, generate Visual Studio solution)
::    Build for VS 2026 using the v143 toolset (i.e. VS 2022 compiler), and target x64 architecture
cmake .. -G "Visual Studio 18 2026" -T v143 -A x64

:: 3. Compile the code using the solution we just generated
cmake --build . --config Release
cmake --build . --config Debug

echo ==========================================
echo Build Complete!
echo Check 'build/Release' for OpenCvPlugin.dll
echo ==========================================
pause