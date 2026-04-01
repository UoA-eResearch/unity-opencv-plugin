@echo off
setlocal enabledelayedexpansion

echo ==========================================
echo  Build Package for Unity OpenCV Plugin
echo ==========================================
echo.

:: -----------------------------------------------
:: 1. Build the native DLL (calls build_windows.bat)
:: -----------------------------------------------
echo [1/4] Building native DLL...
call build_windows.bat
if errorlevel 1 (
    echo ERROR: Native build failed.
    goto :error
)
cd /d "%~dp0"

:: -----------------------------------------------
:: 2. Copy OpenCvPlugin.dll to Package
:: -----------------------------------------------
echo.
echo [2/4] Copying OpenCvPlugin.dll to Package/Plugins/x86_64/...
if not exist "build\Release\OpenCvPlugin.dll" (
    echo ERROR: build\Release\OpenCvPlugin.dll not found. Did the build succeed?
    goto :error
)
copy /Y "build\Release\OpenCvPlugin.dll" "Package\Plugins\x86_64\OpenCvPlugin.dll"

:: -----------------------------------------------
:: 3. Copy opencv_world DLL to Package
:: -----------------------------------------------
echo.
echo [3/4] Copying opencv_world4120.dll to Package/Plugins/x86_64/...

:: Try to find the OpenCV DLL from the CMakeLists.txt OpenCV_DIR path
:: Default location based on CMakeLists.txt: C:/dev/libs/opencv/opencv-4.12.0/build/x64/vc16/bin
set "OPENCV_BIN=C:\dev\libs\opencv\opencv-4.12.0\build\x64\vc16\bin"
set "OPENCV_DLL=%OPENCV_BIN%\opencv_world4120.dll"

if not exist "%OPENCV_DLL%" (
    echo WARNING: opencv_world4120.dll not found at:
    echo   %OPENCV_DLL%
    echo.
    echo Please copy it manually to Package\Plugins\x86_64\
    echo The DLL is typically found in your OpenCV installation's bin directory.
    echo.
) else (
    copy /Y "%OPENCV_DLL%" "Package\Plugins\x86_64\opencv_world4120.dll"
    echo Done.
)

:: -----------------------------------------------
:: 4. Stamp version into package.json
:: -----------------------------------------------
echo.
set /p VERSION="[4/4] Enter version number (e.g. 1.0.0): "
if "%VERSION%"=="" (
    echo No version entered, skipping version stamp.
) else (
    echo Stamping version %VERSION% into Package/package.json...
    powershell -Command "(Get-Content 'Package\package.json' -Raw) -replace '\"version\": \"[^\"]*\"', '\"version\": \"%VERSION%\"' | Set-Content 'Package\package.json' -Encoding UTF8"
)

:: -----------------------------------------------
:: Done
:: -----------------------------------------------
echo.
echo ==========================================
echo  Package build complete!
echo ==========================================
echo.
echo DLLs are in Package\Plugins\x86_64\
echo.
echo Next steps:
echo   1. Test locally: in your Unity project's Packages/manifest.json, add:
echo      "com.github.uoa-eresearch.unity-opencv-plugin": "file:C:/dev/projects/dance/OpenCvPlugin/Package"
echo   2. Open Unity and verify everything compiles and works
echo   3. When satisfied, commit and tag:
echo      git add Package/Plugins/x86_64/*.dll Package/package.json
echo      git commit -m "Release v%VERSION%"
echo      git tag v%VERSION%
echo      git push origin main --tags
echo.
goto :end

:error
echo.
echo Build failed. See errors above.
exit /b 1

:end
endlocal
pause
