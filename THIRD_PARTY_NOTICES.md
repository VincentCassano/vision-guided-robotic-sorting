# Third-party dependencies and references

- **Intel RealSense / pyrealsense2:** camera runtime dependency supplied through the host installation.
- **PyTorch, NumPy, OpenCV and PyYAML:** runtime dependencies installed separately; their own license terms apply.
- **YOLOv5:** model-training and historical checkpoint support. The default deployed path consumes a TorchScript model; this source publication does not bundle the YOLOv5 repository or training datasets.
- **RealMan SDK:** obtained from the robot's existing installation. Vendor SDK binaries are not included in this source publication.
- **OYMotion hand geometry:** the local nominal geometry reference came from `oymotion/rohand_gen2_urdf_ros1`, commit `4385c6c5c381729995c621d09b39182161fc4676`. The supplied MIT notice is retained in `deployment/ROHAND_GEOMETRY_REFERENCE_LICENSE.txt`.
- **School demonstration references:** files under `deployment/school_bin_reference/` retain their supplied source content. Historical test and school-reference provenance is not a certification of a new robot installation.

No new project-wide license is granted by this notice. Existing third-party notices and rights remain applicable. Model weights, vendor binaries, teaching bundles, and on-site calibration/data are managed separately.
