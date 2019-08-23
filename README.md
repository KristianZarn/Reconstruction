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

## Building

coming soon... 


