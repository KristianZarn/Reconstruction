
# SPARSE:
- (low) fix random number generator in theiaSFM
- (low) primerjaj track length, omejitev track length (max 50 npr.)
- multithreading kjer se da
- **reestimate tracks after remove view**
- **prikaži kvaliteto dodane kamere**
    - št. ujemanj, koliko točk vidi, itd.
- **Vizualizacija GSD in/ali degree of redundancy**
- **Napiši svoj reconstruction estimator**
    - večino se da skopirat iz incremental estimator
    - posodobi View razred z unordered_map of <feature, trackid>
- implementiraj AC-RANSAC ?
- remove unestimated views (button)
- update next_image_idx if reconstruction fails

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
- bolj pametno iskanje ujemanj (vocabulary tree, ali kaj drugega)

# NEXT BEST VIEW:
- todo

# OTHER:
- (low) prikaz zajetih slik
    - prikaz še ostalih stvari, kot v COLMAP
- **sprobi zhenhuje dataset z TheiaSFM in OpenMVG**
- pogled na sceno iz kamere (pri lokalizaciji)
- **kamera v ločeni niti**
- zajemanje slik preko telefona: optimizacija (branje iz diska)
- fix show/hide cameras
- crash when removing view and then extend / localize (neki od tega)
- extend with all images