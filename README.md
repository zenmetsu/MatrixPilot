# MatrixPilot 
## Firmware for Bill Premerlani's IMU based UAV Dev Board and its successors

Welcome to the MatrixPilot codebase. 

The UAV Dev Board, originally designed and open sourced by Bill Premerlani, is an IMU-based development platform for building controllers for Unmanned Aerial Vehicles.  This site hosts the development of the MatrixPIlot UAV code that runs on this board, allowing it to stabilize an airplane, maintain altitude, and navigate to specific locations, or back to the launch site.

The [UAV Dev Board from Spark Fun](https://www.sparkfun.com/products/11703) has been retired. The UDB5 is an open source hardware design, and some members of the group are fabricating new boards using [MacroFab](https://macrofab.com/).

You can still buy an AUAV3 Dev Board from Nick Arsov: 
[International Site](http://arsovtech.com/?p=1257).
[USA Site](http://www.auav.co/product-p/auav3.htm)

Projects for the boards are provided for the MPLAB-X Integrated Development Environment (IDE) and XC16 compiler. From October 2016, the master branch no longer supports the legacy MPLAB IDE.

There are 4 projects in the repository. They are listed here in order of increasing complexity:-

* ./Tools/FlashOSD updates the character set of the native On Screen Display (OSD). 
* ./Tools/LedTest is used to test hardware including the accelerometers, gyros, Leds, and PWM outputs.It is factory installed for all 3 boards allowing users to test their boards on delivery.
* ./RollPitchYaw creates an Inertial Measurement Unit (IMU). It  enables higher level testing of algorithms, firmware and our hardware all combined together.
* ./MatrixPilot integrates the IMU with the potential of achieving full automated flight control.

There are two types of simulation avaiable:-
* Software in the Loop (SIL) is provided in ./Tools/MatrixPilot-SIL. The autopilot is simulated as a software process in a host computer (Unix, MacOS, Windows) and that is then, in turn, connected to fly an aircraft in the X-Plane 10 flight simulator.
* Hardware in the Loop (HIL) connects a physical board running MatrixPilot to a simulated aircraft in X-Plane 10. It is enabled for any board by setting a parameter in ./Config/options.h when compiling MatrixPilot. 

./Tools/HilSim provides the software plugin for X-Plane10 to link to the SIL or HIL versions of MatrixPilot. 

./Tools/flight_analyzer provides a python program that allows analysis and display of flights with the help of GoogleEarth, after post-processing with our tool 'flan'.

For developers, there is also a makefile based build system (build-all.bat) and a project generator (Tools/Build/proj_gen.bat), which both Windows and *nix compatible.

For more info about how to configure and use this autopilot firmware, go to the Wiki at:

   https://github.com/MatrixPilot/MatrixPilot/wiki

and see the mailing list at:

   https://groups.google.com/forum/#!forum/uavdevboard
