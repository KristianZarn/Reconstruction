
# SPARSE:
- (low) fix random number generator in theiaSFM
- (low) primerjaj track length, omejitev track length (max 50 npr.)
- multithreading kjer se da
- **prikaži kvaliteto dodane kamere**
    - št. ujemanj, koliko točk vidi, itd.
- **Napiši svoj reconstruction estimator**
    - večino se da skopirat iz incremental estimator
    - posodobi View razred z unordered_map of <feature, trackid>
- (low) implementiraj AC-RANSAC ?
- remove unestimated views (button)
- update next_image_idx if reconstruction fails
- shrani/naloži redko rekonstrukcijo

# DENSE:
- **undistort images**
- densify point cloud (bi blo zelo fino)
- **limit texture resolution**
    - TextureMesh ima samo resolution level možnost
    - texture packing (ni pomoči ?)
    - **Poglej velikost teksture v mvs_scene in zmanjšaj po potrebi**

# EDIT:
- (low) fix crash when mesh cannot be loaded
- (low) fix color on selection
- (low) posodobi uv koordinate po odstranjevanju točk/trikotnikov
- isti mvs objekt za edit in reconstruction (da ni treba save/load)

# LOCALIZATION:
- posodobi
- združi z webcam in IPcamera

# NEXT BEST VIEW:
- nbv based on measure (non linear optimization)

# OTHER:
- (low) prikaz zajetih slik
    - prikaz še ostalih stvari, kot v COLMAP
- pogled na sceno iz kamere (pri lokalizaciji)
- **kamera v ločeni niti**
- crash when removing view and then extend / localize (neki od tega)
- extend with all images
- **speedup colorize**
    - določi barvo glede na eno sliko (tisto ki doda točko)