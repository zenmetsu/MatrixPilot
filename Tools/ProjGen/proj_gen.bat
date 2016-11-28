@echo off

pyProjectCreator.py -r ../.. -n MatrixPilot -t AUAV3 -c Config/Fantasy
pyProjectCreator.py -r ../.. -n MatrixPilot -t AUAV3 -c Config/Grafas
pyProjectCreator.py -r ../.. -n MatrixPilot -t AUAV3 -c Config/Hilsim_Fantasy_Auav3
pyProjectCreator.py -r ../.. -n MatrixPilot -t UDB5  -c Config/Hilsim_Fantasy_Udb5
pyProjectCreator.py -r ../.. -n MatrixPilot -t AUAV3 -c Config/Linea
pyProjectCreator.py -r ../.. -n MatrixPilot -t AUAV3 -c Config/Grobularis
pyProjectCreator.py -r ../.. -n MatrixPilot -t AUAV3 -c Config/E_Glider

goto END:
pyProjectCreator.py -r ../.. -n MatrixPilot -t SIL
pyProjectCreator.py -r ../.. -n MatrixPilot -t PX4
pyProjectCreator.py -r ../.. -n MatrixPilot -t UDB4
pyProjectCreator.py -r ../.. -n MatrixPilot -t UDB5
pyProjectCreator.py -r ../.. -n MatrixPilot -t AUAV3

pyProjectCreator.py -r ../.. -n RollPitchYaw -t PX4
pyProjectCreator.py -r ../.. -n RollPitchYaw -t UDB4
pyProjectCreator.py -r ../.. -n RollPitchYaw -t UDB5
pyProjectCreator.py -r ../.. -n RollPitchYaw -t AUAV3

pyProjectCreator.py -r ../.. -n LedTest -t PX4
pyProjectCreator.py -r ../.. -n LedTest -t UDB4
pyProjectCreator.py -r ../.. -n LedTest -t UDB5
pyProjectCreator.py -r ../.. -n LedTest -t AUAV3

goto END:
:END
pause
