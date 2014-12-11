Desktracking
============

Kinect v2.0 PCL Desktracking


Steps:
1. Download Qt Creator 3.1.2 from http://download.heise.de/software/48a00e8c4801a45b48fe40ac8d371bc6/547de6bc/127608/qt-creator-opensource-windows-x86-3.1.2.exe
2. Clone this repository to your machine
3. Add "build" directory next so the source folder of the project
4. Start Qt Creator and open the .pro (project)file
5. On the left panel click "Projects" (Projekte)
6. activate shadow-build in directory: ..\build
7. Erstellungsschritte: custom schritt hinzufügen
  - Kommando: cmake
  - Argumente: -G"NMake Makefiles JOM" ./../src
  - Arbeitsverzeichnis: %{buildDir}\..\build
8. Make:
  - hier muss jom.exe ausgewählt sein
9. Bereinigung aufklappen
  - argument für make.exe: clean
