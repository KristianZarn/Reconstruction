
# SPARSE:
- fix random number generator in theiaSFM
- primerjaj track length, omejitev track length
- multithreading v theiaSFM
- unique ptrs in reconstruction builder
- **GPU sift**
- **reestimate tracks after remove view**
- pri dodajanju kamere preveri koliko "estimated tracks" vidi.
- **Vizualizacija GSD in/ali degree of redundancy**

# DENSE:
- undistort images
- densify point cloud ?
- **limit texture resolution**
    - TextureMesh ima samo resolution level možnost
    - texture packing (ni pomoči ?)
    - **Poglej velikost teksture v mvs_scene in zmanjšaj po potrebi**
- Refine mesh crash (temple)

# EDIT:
- (low) fix crash when mesh cannot be loaded
- (low) keep biggest connected components
- decimacija
- fix color on selection
- reset mesh

# OTHER:
- **prikaz zajetih slik**
- **lepši prikaz kamer (ne samo pike)**
- **sprobi zhenhuje dataset z TheiaSFM in OpenMVG**

# BIG FEATURES:
- next best view
- realtime camera position

--------------------------------------------------
# PRIORITY:
- undistort images (for mvs)
- izberi več trikotnikov in nato ravnina
    - gumb fit plane
- decimacija in velikost textrure pri exportu
    - recimo dva parametera ki ju nastavi uporabnik pred exportom.
