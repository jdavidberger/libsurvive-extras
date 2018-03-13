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

# Pose / Lighthouse solvers

## Find lighthouse vs find object

The calibration problem can be framed as such: Given an object and the 3d points of its sensors, find the position of the lighthouse from the light data given at each of the visible sensors. 

The pose problem is framed opposite: Given the position of the lighthouse(s), and knowing the rigid structure of the sensors on the tracked object, find the position of the tracked object. 

Typically for calibration it makes sense to assume the tracked object is at origin and solve for the lighthouse that way. This gives us a transform from `world space` to `camera space`. 

When we are tracking, the object is moving and no longer at origin, so now we just are getting `object space` to `camera space`. If we find the inverse of calibration transform, we can chain them together which gives us a transform from `object space` to `world space` which gives us both the position and orientation of the tracked object. 

## PNP Solver

The lighthouse and sensor system fits nicely into the [PNP](https://en.wikipedia.org/wiki/Perspective-n-Point) problem domain.

From the light data, we can derive the angle on each sweep plane from (roughly) the center of the lighthouse to the sensor. Both planes gives us a set of two angles. These angles describe a line in space which intersects both the sensor and the lighthouse. Since both planes are thought of as intersecting a single point, those angles typically fall in the range of [-POV/2, POV/2] -- although the fact that they are centered is somewhat arbitrary and set that way for convienence. 

This tracks loosely with that of a [pinhole camera model](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html). *However*, it does not track exactly. In a typical application of PNP to pinhole cameras, you are given pixels for correspondence whereas here we have angles. This makes the math much simpler. 

Whereas with a pinhole camera, you need to calibrate a camera matrix; which includes center point and focal length. There is no sensor lens in our case, which means there are no pixel values per se. However, if you imagine a plane exactly 1 meter (or whatever unit your 3d points are in) in front of the lighthouse, you can solve for where the line would intercept that plane from the given angle. 

Recall that 

```
tan(angle) = opposite / adjacent
tan(angle) * adjacent = opposite 
```

and since the adjacent line -- that of the center of the plane to the center of the lighthouse -- was imagine at 1 meter, the position for each coordinate is simply:

```
px = tan(ang_x)
py = tan(ang_y)
```

Note that these aren't technically pixel values, but can effectively be used as such. Also note that since we defined our plane at 1m out, this is our focal length and the camera matrix is simply identity. 

From that, we can turn all of our angles into effective 'pixel' values, and use any off the shelf pnp solver. This repo uses OpenCV's for convience. 

## SBA Solver

Bundle adjustment is, loosely speaking, an approach that adjusts the position of the object or the lighthouse and simulates where it expects the resulting angle_x, angle_y measurement to be. Its goal is to adjust the pose of the moving peice to minimize the difference between observed data and the simulated data. 

The cannonical use of SBA in computer vision is take a bunch of pictures, and a bunch of correspondences from the same object in all the pictures, and recreate both the 3d position of objects and the pose of the cameras. 

An important point to SBA is that it needs an OK solution to adjust for good results in most applications. It solves for the local minima of the error function, so to find the global minima it needs to start near it. 

Since SBA models the whole system, the details for calibration and for tracked object position are different. For calibration, you declare the tracked object fixed at origin, and adjust for the position of the two lighthouses. This looks like a N 3d point, 2 camera setup, where N is the number of 3d points on the tracked object.

For tracking, you want to leverage the idea that you know the rigid structure of the tracked object, and the exact position of the lighthouses, and you just want the pose of the tacked object. The way this is modeled here is that there is one pose for the tracked object, and effectively two cameras for each sensor location, all at known positions that aren't to be adjusted. 

Both approaches use the same reprojection function which is part of libsurvive.

SBA buys us a few things in this domain. Primarilly, it incorporates data from all correspondences and all lighthouses; something that PNP doesn't really do out of the box. But more importantly, it lets us model the _actual_ parametrics of the lighthouse. The lighthouse is _not_ a pinhole camera, but treating it as one gets you arbitrarilly close. SBA typically will optimize close into the optimal least squares solution. This is going to be important in the long run, paticurally since each lighthouse has factory calibration parameters that are almost certainly easier to reproject than any other approach. 

The SBA library used here is the [http://users.ics.forth.gr/~lourakis/sba/](one here). Performance wise, since the problem is so small, I haven't implemented a jacobian for the reprojection function -- it runs as fast as it can get light data. If it needs to run on a much slower platform, a jacobian would speed it up substantially. 
