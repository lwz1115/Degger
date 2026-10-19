@echo off
title 无人船调试助手
cd /d "C:\Users\Administrator\Desktop\G12Debugger"
set "PYW=C:\Users\Administrator\AppData\Local\Programs\Python\Python312\pythonw.exe"
if not exist "%PYW%" set "PYW=pythonw"
start "" "%PYW%" "C:\Users\Administrator\Desktop\G12Debugger\app\usv_debug.py" %*
