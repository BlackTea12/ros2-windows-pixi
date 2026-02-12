@echo off
@REM IMPORTANT NOTE: it must be MSVC 2022 version
call "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat" -arch=x64

@REM move to your workspace
cd /d D:\Workspace\pixi_ws\ros2-windows

@REM activate jazzy environment
pixi shell -e jazzy
