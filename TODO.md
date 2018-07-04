
# SPARSE:
- fix random number generator in theiaSFM
- primerjaj track length, omejitev track length
- multithreading v theiaSFM
- unique ptrs in reconstruction builder
- **reestimate tracks after remove view**
- pri dodajanju kamere preveri koliko "estimated tracks" vidi.
- **Vizualizacija GSD in/ali degree of redundancy**
- **Napiši svoj reconstruction estimator**
    - večino se da skopirat iz incremental estimator
- implement AC-RANSAC ?
- remove unestimated views

# DENSE:
- undistort images
- densify point cloud ?
- **limit texture resolution**
    - TextureMesh ima samo resolution level možnost
    - texture packing (ni pomoči ?)
    - **Poglej velikost teksture v mvs_scene in zmanjšaj po potrebi**

# EDIT:
- (low) fix crash when mesh cannot be loaded
- (low) keep biggest connected components
- (low) fix color on selection
- (low) update uv's on delete
- polnjenje lukenj (po odstanjevanju ravnine npr.)

# OTHER:
- **prikaz zajetih slik**
- **lepši prikaz kamer (ne samo pike)**
- **sprobi zhenhuje dataset z TheiaSFM in OpenMVG**
- hide all možnost

# BIG FEATURES:
- next best view
- realtime camera position

--------------------------------------------------
# PRIORITY:
- undistort images (for mvs)
- velikost textrure
- uv koordinate po odstranjevanju točk/trikotnikov ?