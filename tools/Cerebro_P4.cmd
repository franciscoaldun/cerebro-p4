@echo off
title Cerebro P4 - puente (camara del PC -^> P4 -^> webcam con IA)
cd /d "C:\Users\USUARIO\esp\projects\p4-webcam\tools"
set PYTHONIOENCODING=utf-8
echo Conecta el P4 por el puerto HUSB. Dashboard: http://192.168.7.1
python puente.py --abrir
pause
