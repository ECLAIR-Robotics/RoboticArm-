set venv_name=venv
set idle_script_name=idle

@REM since running as administrator will run this in C:\Windows\System32
cd %~dp0

@REM download and install python
curl.exe -o python-3.10.4-install.exe https://www.python.org/ftp/python/3.10.4/python-3.10.4-amd64.exe
.\python-3.10.4-install.exe
del /f python-3.10.4-install.exe


@REM create python venv and pip install libraries
python -m venv ..\%venv_name%
..\%venv_name%\Scripts\pip3.exe install pyserial
..\%venv_name%\Scripts\pip3.exe install opencv-python

@REM create IDLE start script
echo ..\%venv_name%\Scripts\activate.bat > "..\%idle_script_name%.bat"
echo python -m idlelib.idle .\code\main.py >> ..\%idle_script_name%.bat
