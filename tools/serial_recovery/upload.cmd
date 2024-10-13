@echo off
echo Available USB Serial COM ports:
wmic path CIM_LogicalDevice where "Description like 'USB Serial Port%'" get Caption
set /p port="Select port of your device (like COMx): "
echo:
echo Recovery tool version:
.\tool\mcumgr.exe version
echo:
echo Current device image list:
.\tool\mcumgr.exe --conntype serial --connstring "%port%" image list
echo Upload...
.\tool\mcumgr.exe --conntype serial --connstring dev=%port% image upload -e .\bin\app_update.bin
pause
echo New device image list:
.\tool\mcumgr.exe --conntype serial --connstring "%port%" image list
pause