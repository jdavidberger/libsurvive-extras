# libsurvive-extras

This library extends [https://github.com/cnlohr/libsurvive](libsurvive) with experimental algorithms and vizualizers.

The poser algorithms are slated to be backported in some form or another. The visualization tools likely are not. 

# Getting started

To build the software, do the following:

```
git clone https://github.com/jdavidberger/libsurvive-extras.git
cd libsurvive-extras
mkdir build
cd build
cmake ..
make -j4
```

The first time you do this on your system, it will download and build dependencies which is unfortunately slow right now -- the main large ones are opencv and POCO.

You will also need to have lapack installed on your system seperatatly.

Once it is built, you can plug in your vive gear and run the server:

```
make run_server
```

if you aren't running make, all this does is run `server_main` with the first argument pointing to the directory at `src/viz/server/web`. 

This will start a server you can browse to with a webgl enabled browser: [http://localhost:8080](http://localhost:8080). 

If you are running it on a system that doesn't have a monitor, you can access it from another computer by replacing 'localhost' with the target machines IP address. 

When libsurvive is done solving the calibration, the lighthouses and tracked object should appear on screen. Note that lighthouses, by convention, have their Z axis facing _away_ from the direction they are pointing. Faint field of view cones are provided to visualize what the lighthouses are seeing.

# Video

[Lighthouse tracking a watchman controller](https://youtu.be/jPOV6_VuduI)
