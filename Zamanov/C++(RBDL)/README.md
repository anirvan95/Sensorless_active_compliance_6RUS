Installing RBDL and LUA
-----------------------
1. Install Eigen3 from source (http://eigen.tuxfamily.org/index.php?title=Main_Page)
2. Install Lua (sudo apt-get install lua5.1-dev)
3. Install RBDL from source (https://bitbucket.org/rbdl/rbdl/src), also make sure to enable the Lua model addon (Add the flag "-D RBDL_BUILD_ADDON_LUAMODEL=ON" during cmake)
4. Further dependencies (may not be required) - *libqt4-dev and *libboost-all-dev

Installing Dynamixel libraries
------------------------------
Runtime libraries (C, Linux x64) for Dynamixel can be found at libs/ and the includes at dynamixel_sdk_c/, however if you choose a different language binding or platform than the one provided, follow the following steps.
1. Clone the source of Dynamixel SDK from github (https://github.com/ROBOTIS-GIT/DynamixelSDK.git)
2. Go to your preferred language binding folder ( 'cd c' for e.g.)
3. Go to to your preferred OS platform ( 'cd linux64' )
4. Type 'make' to build the libraries. Next, you may either wish to install them to your standard paths (use 'sudo make install'), or copy them to your working directory (under 'libs/'). Add the directory containing the copied .so (or .dll) file to your linker path in CMakeLists.txt
5. Copy the includes at <language>/include/ to your working directory. Add the path of this copied directory to your include paths in CMakeLists.txt

How to execute Stewart Zamanov 
-------------
1. Create a build folder
2. Enter into the build folder and execute cmake ../
3. Execute make
4. copy the Zamanov_mechanism_dhs.lua into the build folder
5. execute ./zamanov_stewart


Current Issues
--------------
1. Integrate Dynamixel_SDK c++ along with current project 
	- DynamixelSDK can be cloned from (https://github.com/ROBOTIS-GIT/DynamixelSDK)
	- Installation manual (http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/cpp_linux/#cpp-linux)
