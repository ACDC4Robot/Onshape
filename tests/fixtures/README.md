# Native Onshape 1.222 fixtures

Created in the user account for ACDC4Robot testing on 26 October 2026.
Original native archives are retained byte-for-byte. Both use binary STL,
medium resolution, Aluminum 6061, and a 10 mm extrusion.

The articulated fixture repeats the same part twice, fixes the first instance,
and uses a revolute mate about Z with limits -45 and +60 degrees. The child
mate connector has an X offset of -80 mm; alignment places its solid at +80 mm.
Onshape exports one mesh reused by two physical links, two tiny positive-mass
mate-connector links, and a geometry-free massless root. These source values
are retained, including effort/velocity values supplied by Onshape.

Source document: https://cad.onshape.com/documents/e4229ddb96cfbdeede9a3028/w/53f4f3831ef3668f7ec0bec4/e/f50ad86f0f38bf15210a64c6
Workspace is mutable; archive hashes are the identity of the tested data.
No third-party Barkour CAD is bundled.
