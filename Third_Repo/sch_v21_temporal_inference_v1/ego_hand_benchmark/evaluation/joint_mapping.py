import numpy as np

# SOMA's *1 joints for non-thumb fingers are proximal metacarpal nodes,
# not MANO MCPs. Match names as well as indices to catch asset changes.
SOMA25_TO_HAND21 = np.array(
    [0, 1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24]
)
SOMA_SUFFIXES = (
    "",
    "Thumb1",
    "Thumb2",
    "Thumb3",
    "ThumbEnd",
    "Index1",
    "Index2",
    "Index3",
    "Index4",
    "IndexEnd",
    "Middle1",
    "Middle2",
    "Middle3",
    "Middle4",
    "MiddleEnd",
    "Ring1",
    "Ring2",
    "Ring3",
    "Ring4",
    "RingEnd",
    "Pinky1",
    "Pinky2",
    "Pinky3",
    "Pinky4",
    "PinkyEnd",
)


def verify_soma_names(names, hand):
    expected = [hand.capitalize() + "Hand" + suffix for suffix in SOMA_SUFFIXES]
    if list(names) != expected:
        raise ValueError(
            f"Unexpected {hand} SOMA joint order; update the explicit anatomical mapping"
        )


def soma_camera_joints(joints, translation):
    joints = np.asarray(joints)
    translation = np.asarray(translation)
    if joints.shape[-2:] != (25, 3) or translation.shape != joints.shape[:-2] + (3,):
        raise ValueError("Expected SOMA25 wrist-local joints and matching camera translations")
    return joints[..., SOMA25_TO_HAND21, :] + translation[..., None, :]


def wilor_camera_joints(joints, translation, hand):
    """WiLoR already uses OpenPose21; mirror the canonical right hand once."""
    joints = np.array(joints, copy=True)
    if joints.shape != (21, 3) or hand not in ("left", "right"):
        raise ValueError("Expected WiLoR OpenPose21 and an anatomical hand side")
    if hand == "left":
        joints[:, 0] *= -1
    return joints + np.asarray(translation)
