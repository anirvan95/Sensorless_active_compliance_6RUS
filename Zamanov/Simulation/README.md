Installing RBDL and LUA
-----------------------
1. Install Eigen3 from source (http://eigen.tuxfamily.org/index.php?title=Main_Page)
2. Install Lua (sudo apt-get install lua5.1-dev)
3. Install RBDL from source (https://bitbucket.org/rbdl/rbdl/src), also make sure to enable the Lua model addon (Add the flag "-D RBDL_BUILD_ADDON_LUAMODEL=ON" during cmake)
4. Further dependencies (may not be required) - *libqt4-dev and *libboost-all-dev
5. Install MeshUp (https://github.com/ORB-HD/MeshUp)

Building the Simulation Files
-----------------------------
make
./zamanov_mechanism (run the executable to generate the csv files)
meshup Zamanov_mechanism_dhs.lua Zamanov_mechanism_animation.csv (check file name generated)

Current Issue 
-------------
Copy test_base.obj in /usr/**/meshup/meshes
