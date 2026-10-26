# Shared ACDC4Robot code

`shared/convexify_urdf.py` and `shared/smoke_test_isaac_urdf.py` are independent
module snapshots, with recorded companion-specific adaptations, from ACDC4Robot/Fusion360 at commit
`15a5f80` (the complete commit hash is recorded in `shared-source.json`).
The decimal formatter in `serialization.py` is adapted from the same
repository's `Add-IN/ACDC4Robot/core/serialization.py`.

Source: https://github.com/ACDC4Robot/Fusion360

Copyright (c) 2023 BionicDL@SUSTech. These files are used under the MIT License
included in `LICENSE`; its copyright, permission, and warranty notices apply.
The Onshape package does not import the Fusion Add-In or require Autodesk APIs.

Optional dependencies retain their own licenses. Installing an optional extra
does not grant access to Onshape or NVIDIA services.
