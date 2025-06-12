#!/usr/bin/env python3
"""
Undistort a fisheye video (with optional border crop / resize) **and** save the
processed stream to disk.

• Set SAVE_VIDEO → True to write an MP4.
• Choose whether to export only the undistorted view or a side-by-side preview.
"""

import cv2, yaml, numpy as np, os
from pathlib import Path

# ─────────── USER SETTINGS ───────────────────────────────────────
VIDEO = Path('/home/amither/Downloads/2025-05-13_14-55-16.mp4')
YAML          = Path('cam_calib.yaml')
BALANCE       = 0.00          # 0=crop hard, 1=keep full fisheye

SIDE_BY_SIDE  = False         # export “orig | undist” instead of only undistorted

SAVE_VIDEO    = True
OUT_PATH = VIDEO.with_name(VIDEO.stem + '_undist' + VIDEO.suffix)
FOURCC        = 'mp4v'        # good cross-platform default


# ─────────────────────────────────────────────────────────────────

# ─── read first frame to discover resolution & fps ──────────────
cap = cv2.VideoCapture(str(VIDEO))
ok, first = cap.read()
if not ok:
    raise RuntimeError(f'Cannot read from {VIDEO}')
h_vid, w_vid   = first.shape[:2]
fps            = cap.get(cv2.CAP_PROP_FPS) or 30.0  # fallback if FPS not stored

# ─── load YAML calibration and upscale intrinsics to video size ─
with open(YAML) as f:
    data = yaml.safe_load(f)

orig_w = data.get('image_width', 640)
orig_h = data.get('image_height', 480)

K_yaml = np.array([[data['camera_matrix']['fx'], 0, data['camera_matrix']['cx']],
                   [0, data['camera_matrix']['fy'], data['camera_matrix']['cy']],
                   [0, 0, 1]], dtype=np.float64)

sx, sy = w_vid / orig_w, h_vid / orig_h
K      = K_yaml.copy()
K[0, 0] *= sx;  K[1, 1] *= sy           # fx, fy
K[0, 2] *= sx;  K[1, 2] *= sy           # cx, cy

D = np.asarray(data['distortion_coefficients']['data'],
               dtype=np.float64).reshape(-1, 1)

# Decide whether YAML represents a fisheye (4-coef) or pinhole r-tan model
FISHEYE_MODEL = (data.get('distortion_model', 'fisheye') == 'fisheye'
                 or len(D) == 4)      # 4 coeffs → typical fisheye file

# ─── build undistort-rectify maps, fisheye OR pinhole ──────────
if FISHEYE_MODEL:
    # fisheye model (OpenCV equidistant)
    R = np.eye(3)
    Knew = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
               K, D, (w_vid, h_vid), R,
               balance=BALANCE, fov_scale=1.0)
    print(Knew)

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                    K, D, R, Knew, (w_vid, h_vid), cv2.CV_16SC2)
else:
    # pinhole + radial–tangential (r-tan) model
    alpha = 1.0 - BALANCE          # same dial: 0=crop hard, 1=keep FOV
    Knew, _ = cv2.getOptimalNewCameraMatrix(
                  K, D, (w_vid, h_vid), alpha, (w_vid, h_vid))

    map1, map2 = cv2.initUndistortRectifyMap(
                    K, D, None, Knew, (w_vid, h_vid), cv2.CV_16SC2)

# ─── VideoWriter (opened lazily once first processed frame is ready) ───
writer = None
def _open_writer(frame_shape):
    global writer
    if not SAVE_VIDEO:
        return
    if writer is None:            # open once
        h, w = frame_shape[:2]
        if SIDE_BY_SIDE:
            w *= 2
        fourcc = cv2.VideoWriter_fourcc(*FOURCC)
        OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(str(OUT_PATH), fourcc, fps, (w, h))
        if not writer.isOpened():
            raise RuntimeError(f'Could not open {OUT_PATH} for writing')

# ─── processing loop ─────────────────────────────────────────────
frame_idx = 0
while True:
    ok, frm = cap.read()
    if not ok:
        break
    und = cv2.remap(frm, map1, map2, cv2.INTER_CUBIC)

    # ── open writer lazily with the final dimensions ────────────
    _open_writer(und.shape)

    # ── compose side-by-side preview if requested ───────────────
    out_frame = np.hstack((frm, und)) if SIDE_BY_SIDE else und
    preview = out_frame

    # ── write & preview ─────────────────────────────────────────
    if SAVE_VIDEO:
        writer.write(out_frame)

    cv2.imshow('orig | undist', preview)
    if cv2.waitKey(1) & 0xFF in (27, ord('q')):
        break

    frame_idx += 1

# ─── cleanup ────────────────────────────────────────────────────
cap.release()
if writer:
    writer.release()
cv2.destroyAllWindows()
print(f'✅ Finished. {frame_idx} frames processed.')
if SAVE_VIDEO:
    print(f'📼 Saved → {OUT_PATH}')