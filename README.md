# Online reconstruction with next best view planning

This software entire supports reconstruction process, from image acquisition to dense reconstruction, with big help from libraries such as [TheiaSfM](https://github.com/sweeneychris/TheiaSfM) and [OpenMVS](https://github.com/cdcseacave/openMVS).

Images can be loaded from disk or captured through IP camera. The user can get online information about the adequacy of every captured image. An estimate of quality for the current 3D model is also shown to help with further image acquisition.

A prototypical method for next best view planning, that systematically improves the quality of reconstruction, is also implemented.

## Functionalities

Functionalities of this software are split into modules that contain related functions.
The modules can be turned on or off as needed.

### Screenshot of UI

Screenshot of the entire UI is shown below. The modules are marked with red border.

![UI](https://github.com/KristianZarn/Reconstruction/blob/master/screenshots/uporabniski_vmesnik.png)

### Module for image acquisition

Images can be loaded from disk or captured with IP camera.
Button "Lokaliziraj sliko" checks if the image can be added to reconstruction.
If localization is successful a green colored camera is placed in the scene at appropriate location.

![images](https://github.com/KristianZarn/Reconstruction/blob/master/screenshots/modul_slike.png)

### Module for reconstruction

This module offers functions for sparse and dense reconstruction. 

![reconstruction](https://github.com/KristianZarn/Reconstruction/blob/master/screenshots/modul_rekonstrukcija.png)

### Module for editing

Before dense reconstruction it is desirable to remove unnecessary triangles, to speed up the process.
This module offers some tool to do that.

![editing](https://github.com/KristianZarn/Reconstruction/blob/master/screenshots/modul_urejanje.png)

### Module for next best view

Our prototypical method for NBV is contained in seperate module.
The user can pick the region of interest. Blue colored camera is placed on position suggested for next image capture.

![images](https://github.com/KristianZarn/Reconstruction/blob/master/screenshots/modul_nbv.png)

## Usage

- `main_disk.cpp` is used to run the software when loading the images from disk.
- `main_ip_camera` is used to run the software with IP camera image acquisition.
- Other `main` files were used mainly for evaluation and testing.


- **Dataset for testing**:
- https://drive.google.com/file/d/1IzgAXKMrH0Ev4cJcoHZmy3kIZGnyv3EP/view?usp=sharing

-------------------------------------------------------------------------------------
## Building


General notes:
- Build was done on Ubuntu 22.04 machine.
- Main dependencies needed are TheiaSfM, COLMAP, and OpenMVS with CUDA support.
- Library package that was used for the build is available on this link. Provided libraries have certain modifications listed below.
- All of the main libraries should be build using cmake. Use Unix makefiles generator for CMake when building the libraries. Other notes on the libraries are listed below.

**Use this library package**:
- https://drive.google.com/file/d/13NWQlFV2arv1KXhpP_ZiEkobELpGZysA/view?usp=sharing

Prerequisites:
- sudo apt install build-essential
- sudo apt install cmake
- sudo apt install libglm-dev
- sudo apt install libflann-dev

### CUDA 11.7:

- Install CUDA Toolkit 11.7 from nvidia website (CUDA Toolkit 11.5 from package manager will not work)

- Post install actions: https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#post-installation-actions
    - Add these line to your ~/.bashrc and reload the terminal
    - export CUDA_HOME=/usr/local/cuda
    - export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64
    - export PATH=$PATH:$CUDA_HOME/bin

- test with: nvcc -V

### TheiaSfM 0.8:

- sudo apt install libeigen3-dev
- sudo apt install libcurl4-openssl-dev
- sudo apt install libceres-dev
- sudo apt install libgflags-dev
- sudo apt install libopenimageio-dev
- sudo apt install librocksdb-dev
- sudo apt install freeglut3-dev

Other Changes and potential problems:
- Use c++14 compiler flag because of ceres version 2
- apply this patch: https://github.com/sweeneychris/TheiaSfM/issues/261
- Some custom code was added to the library so use the provided package.

### COLMAP 3.5:

- sudo apt install libboost-all-dev
- sudo apt install libfreeimage-dev
- sudo apt install libglew-dev
- sudo apt install qtbase5-dev
- sudo apt install libcgal-dev

Other Changes and potential problems:
- Use c++14 compiler flag because of ceres
- you have to see "CUDA support enabled" in cmake output!
- add include "cstdio" v "lib/PoissonRecon/Geometry.h"
- https://github.com/vlfeat/vlfeat/issues/214
- https://bytemeta.vip/repo/colmap/colmap/issues/1418
- https://stackoverflow.com/questions/42504592/flann-util-serialization-h-class-stdunordered-mapunsigned-int-stdvectorun

### OpenMVS 2.0.1:

- sudo apt install libopencv-dev
- download vcg library (header only) and set path in cmake configuration

Other Changes and potential problems:
- https://github.com/NVlabs/instant-ngp/issues/119
- https://forums.developer.nvidia.com/t/cuda-11-6-0-with-gcc-11-2-1-fails-to-process-system-headers-included-by-functional/203556
