W tym repo znajdują się 2 aplikacje jedna do kotroli robota, a druga to interfejs urzytkownika.

Przygotowanie środowiska pod uruchomienie aplikacji na robocie:
Linux:
- python3 -m venv .venv_robot
- source .venv_robot/bin/activate
- pip install -r robot_app/requirements.txt
- pip install -e scara_lib

Windows: 
 - XD

Przygotowanie środowiska pod uruchomienie aplikacji interfesju:
Linux:
- python3 -m venv .venv_ui
- source .venv_ui/bin/activate
- pip install -r simulator_app/requirements.txt
- pip install -e scara_lib

Windows: 
- ...
