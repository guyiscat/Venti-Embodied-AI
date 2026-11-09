@echo off
setlocal
set "PROJECT=%~dp0..\.."
set "PYTHON_EXE=python"
if exist "%PROJECT%\.venv\Scripts\python.exe" set "PYTHON_EXE=%PROJECT%\.venv\Scripts\python.exe"
"%PYTHON_EXE%" "%PROJECT%\code\view_model.py" --model models/Venti_MJCF/venti_skin.xml %*
exit /b %errorlevel%
