
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
- undistort images
- densify point cloud ?
- **limit texture resolution**
    - TextureMesh ima samo resolution level možnost
    - texture packing (ni pomoči ?)
    - **Poglej velikost teksture v mvs_scene in zmanjšaj po potrebi**

# EDIT:
- (low) fix crash when mesh cannot be loaded
- (low) fix color on selection
- (low) update uv's on delete
- polnjenje lukenj (po odstanjevanju ravnine npr.)
- isti mvs objekt za edit in reconstruction (da ni treba save/load)

# LOCALIZATION:
- bolj pametno iskanje ujemanj (vocabulary tree, ali kaj drugega)
    - ideja: vocabulary tree samo pri lokalizaciji iz nič, drugače pa iskanje ujemanj z najbližjo sliko
    - za začetek lahko matching z vsemi slikami namesto vocabulary tree
- Localize image:
    - match with given views
    - match with all views

# NEXT BEST VIEW:
- todo

# OTHER:
- hide/show all možnost
- prikaz zajetih slik
- **sprobi zhenhuje dataset z TheiaSFM in OpenMVG**
- **pogled na sceno iz kamere (pri lokalizaciji)**

--------------------------------------------------
# PRIORITY:
- undistort images (for mvs)
- velikost textrure
- uv koordinate po odstranjevanju točk/trikotnikov ?
- lasten reconstruction estimator
- "pametno" iskanje ujemanj pri lokalizaciji