#!/usr/bin/env python3

import argparse
import time
import os
import yaml
from typing import Dict, Any

import cv2
import numpy as np


# Map human-friendly names to OpenCV ArUco dictionary enums
ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": getattr(cv2.aruco, "DICT_APRILTAG_16h5", None),
    "DICT_APRILTAG_25h9": getattr(cv2.aruco, "DICT_APRILTAG_25h9", None),
    "DICT_APRILTAG_36h10": getattr(cv2.aruco, "DICT_APRILTAG_36h10", None),
    "DICT_APRILTAG_36h11": getattr(cv2.aruco, "DICT_APRILTAG_36h11", None),
}


def _safe_set(dp, name: str, value):
    if hasattr(dp, name):
        setattr(dp, name, value)


def _params_to_dict(dp) -> Dict[str, Any]:
    keys = [
        "adaptiveThreshWinSizeMin",
        "adaptiveThreshWinSizeMax",
        "adaptiveThreshWinSizeStep",
        "adaptiveThreshConstant",
        "useAruco3Detection",
        "minMarkerPerimeterRate",
        "maxMarkerPerimeterRate",
        "polygonalApproxAccuracyRate",
        "minCornerDistanceRate",
        "minDistanceToBorder",
        "minOtsuStdDev",
        "cornerRefinementMethod",
        "cornerRefinementWinSize",
        "cornerRefinementMaxIterations",
        "cornerRefinementMinAccuracy",
        "markerBorderBits",
        "perspectiveRemovePixelPerCell",
        "perspectiveRemoveIgnoredMarginPerCell",
        "maxErroneousBitsInBorderRate",
        "errorCorrectionRate",
        "minMarkerDistanceRate",
    ]
    out = {}
    for k in keys:
        if hasattr(dp, k):
            out[k] = getattr(dp, k)
    return out


def _dict_to_params(d: Dict[str, Any], dp):
    for k, v in d.items():
        _safe_set(dp, k, v)


def build_trackbar_window(win: str):
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    # Integer params
    cv2.createTrackbar("th_win_min", win, 3, 51, lambda v: None)
    cv2.createTrackbar("th_win_max", win, 23, 151, lambda v: None)
    cv2.createTrackbar("th_win_step", win, 10, 50, lambda v: None)
    cv2.createTrackbar("th_const", win, 7, 50, lambda v: None)
    cv2.createTrackbar("useAruco3", win, 0, 1, lambda v: None)
    cv2.createTrackbar("min_dist_border", win, 3, 50, lambda v: None)
    cv2.createTrackbar("min_otsu_std", win, 5, 100, lambda v: None)
    cv2.createTrackbar("cr_method", win, 1, 2, lambda v: None)  # 0: NONE, 1: SUBPIX, 2: APRILTAG_FAST
    cv2.createTrackbar("cr_win", win, 5, 20, lambda v: None)
    cv2.createTrackbar("cr_max_iter", win, 30, 200, lambda v: None)
    cv2.createTrackbar("marker_border_bits", win, 1, 3, lambda v: None)
    cv2.createTrackbar("dict_idx", win, 4, len([k for k, v in ARUCO_DICTS.items() if v is not None]) - 1, lambda v: None)

    # Float params use scaling (x1000)
    cv2.createTrackbar("min_perim_x1000", win, int(0.03 * 1000), 500, lambda v: None)
    cv2.createTrackbar("max_perim_x1000", win, int(4.0 * 1000), 10000, lambda v: None)
    cv2.createTrackbar("poly_rate_x1000", win, int(0.03 * 1000), 200, lambda v: None)
    cv2.createTrackbar("min_corner_dist_x1000", win, int(0.05 * 1000), 500, lambda v: None)
    cv2.createTrackbar("cr_min_acc_x1000", win, int(0.01 * 1000), 200, lambda v: None)
    cv2.createTrackbar("pers_px_per_cell", win, 8, 50, lambda v: None)
    cv2.createTrackbar("pers_ignored_margin_x1000", win, int(0.13 * 1000), 500, lambda v: None)
    cv2.createTrackbar("max_err_bits_border_x1000", win, int(0.35 * 1000), 1000, lambda v: None)
    cv2.createTrackbar("err_corr_rate_x1000", win, int(0.6 * 1000), 2000, lambda v: None)
    cv2.createTrackbar("min_marker_dist_rate_x1000", win, int(0.05 * 1000), 1000, lambda v: None)


def get_trackbar_params(win: str, dp):
    # Integer/direct
    th_min = max(3, cv2.getTrackbarPos("th_win_min", win) | 1)  # force odd
    th_max = max(th_min + 2, cv2.getTrackbarPos("th_win_max", win) | 1)
    th_step = max(1, cv2.getTrackbarPos("th_win_step", win))
    th_const = cv2.getTrackbarPos("th_const", win)
    use_aruco3 = cv2.getTrackbarPos("useAruco3", win)
    min_dist_border = cv2.getTrackbarPos("min_dist_border", win)
    min_otsu_std = cv2.getTrackbarPos("min_otsu_std", win)
    cr_method = cv2.getTrackbarPos("cr_method", win)
    cr_win = cv2.getTrackbarPos("cr_win", win)
    cr_max_iter = cv2.getTrackbarPos("cr_max_iter", win)
    marker_border_bits = cv2.getTrackbarPos("marker_border_bits", win)

    # Scaled floats
    min_perim = cv2.getTrackbarPos("min_perim_x1000", win) / 1000.0
    max_perim = max(min_perim + 0.01, cv2.getTrackbarPos("max_perim_x1000", win) / 1000.0)
    poly_rate = cv2.getTrackbarPos("poly_rate_x1000", win) / 1000.0
    min_corner_dist = cv2.getTrackbarPos("min_corner_dist_x1000", win) / 1000.0
    cr_min_acc = cv2.getTrackbarPos("cr_min_acc_x1000", win) / 1000.0
    pers_px_per_cell = max(1, cv2.getTrackbarPos("pers_px_per_cell", win))
    pers_ignored_margin = cv2.getTrackbarPos("pers_ignored_margin_x1000", win) / 1000.0
    max_err_bits_border = cv2.getTrackbarPos("max_err_bits_border_x1000", win) / 1000.0
    err_corr_rate = cv2.getTrackbarPos("err_corr_rate_x1000", win) / 1000.0
    min_marker_dist_rate = cv2.getTrackbarPos("min_marker_dist_rate_x1000", win) / 1000.0

    _safe_set(dp, "adaptiveThreshWinSizeMin", int(th_min))
    _safe_set(dp, "adaptiveThreshWinSizeMax", int(th_max))
    _safe_set(dp, "adaptiveThreshWinSizeStep", int(th_step))
    _safe_set(dp, "adaptiveThreshConstant", float(th_const))
    _safe_set(dp, "useAruco3Detection", bool(use_aruco3))
    _safe_set(dp, "minMarkerPerimeterRate", float(min_perim))
    _safe_set(dp, "maxMarkerPerimeterRate", float(max_perim))
    _safe_set(dp, "polygonalApproxAccuracyRate", float(poly_rate))
    _safe_set(dp, "minCornerDistanceRate", float(min_corner_dist))
    _safe_set(dp, "minDistanceToBorder", int(min_dist_border))
    _safe_set(dp, "minOtsuStdDev", float(min_otsu_std))
    _safe_set(dp, "cornerRefinementMethod", int(cr_method))
    _safe_set(dp, "cornerRefinementWinSize", int(cr_win))
    _safe_set(dp, "cornerRefinementMaxIterations", int(cr_max_iter))
    _safe_set(dp, "cornerRefinementMinAccuracy", float(cr_min_acc))
    _safe_set(dp, "markerBorderBits", int(marker_border_bits))
    _safe_set(dp, "perspectiveRemovePixelPerCell", int(pers_px_per_cell))
    _safe_set(dp, "perspectiveRemoveIgnoredMarginPerCell", float(pers_ignored_margin))
    _safe_set(dp, "maxErroneousBitsInBorderRate", float(max_err_bits_border))
    _safe_set(dp, "errorCorrectionRate", float(err_corr_rate))
    _safe_set(dp, "minMarkerDistanceRate", float(min_marker_dist_rate))

    return {
        "th_min": th_min,
        "th_max": th_max,
        "th_const": th_const,
    }


def draw_info(img, dict_name: str, det_count: int, fps: float):
    h, w = img.shape[:2]
    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (w, 60), (0, 0, 0), -1)
    alpha = 0.4
    img[:] = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)
    cv2.putText(img, f"Dict: {dict_name}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, f"Detections: {det_count}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(img, f"FPS: {fps:.1f}", (200, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description="Interactive ArUco detector parameter tuner with OpenCV")
    parser.add_argument("--source", type=int, default=0, help="cv2.VideoCapture index (default: 0)")
    parser.add_argument("--width", type=int, default=1280, help="Requested capture width")
    parser.add_argument("--height", type=int, default=720, help="Requested capture height")
    parser.add_argument("--load", type=str, default=None, help="Load parameters from YAML file")
    parser.add_argument("--save", type=str, default="aruco_params_tuned.yaml", help="Save YAML output path")
    parser.add_argument("--dict", type=str, default="DICT_5X5_50", help="Initial ArUco dictionary name")
    args = parser.parse_args()

    available_names = [k for k, v in ARUCO_DICTS.items() if v is not None]
    if args.dict not in available_names:
        args.dict = available_names[0]

    dict_idx = available_names.index(args.dict)

    # Create window and trackbars
    win = "Aruco Tuner"
    build_trackbar_window(win)
    cv2.setTrackbarPos("dict_idx", win, dict_idx)

    # Initialize ArUco
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[available_names[dict_idx]])
    dp = cv2.aruco.DetectorParameters()

    # Load parameters if provided
    if args.load and os.path.isfile(args.load):
        with open(args.load, "r") as f:
            loaded = yaml.safe_load(f) or {}
        _dict_to_params(loaded, dp)

    # Initialize capture
    cap = cv2.VideoCapture(args.source)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print("ERROR: Could not open video source", args.source)
        return 1

    last_time = time.time()
    fps = 0.0

    print("Controls: s=save yaml, n/p=switch dict, q/ESC=quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Update dictionary from trackbar or via keyboard
        new_dict_idx = cv2.getTrackbarPos("dict_idx", win)
        new_dict_idx = max(0, min(new_dict_idx, len(available_names) - 1))
        if new_dict_idx != dict_idx:
            dict_idx = new_dict_idx
            dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[available_names[dict_idx]])

        # Apply parameters from UI
        th_cfg = get_trackbar_params(win, dp)

        # Detect
        detector = cv2.aruco.ArucoDetector(dictionary, dp)
        corners, ids, _ = detector.detectMarkers(frame)

        # Draw detections
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            det_count = len(ids)
        else:
            det_count = 0

        # Compute FPS
        now = time.time()
        dt = now - last_time
        last_time = now
        fps = 0.9 * fps + 0.1 * (1.0 / dt) if dt > 1e-6 else fps

        # Overlay info
        draw_info(frame, available_names[dict_idx], det_count, fps)

        # Adaptive threshold debug views
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        try:
            th1 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, th_cfg["th_min"], th_cfg["th_const"])
            th2 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, th_cfg["th_max"], th_cfg["th_const"])
        except Exception:
            th1 = gray
            th2 = gray

        cv2.imshow(win, frame)
        cv2.imshow("Thresh (min win)", th1)
        cv2.imshow("Thresh (max win)", th2)
        key = cv2.waitKey(1) & 0xFF

        if key in (27, ord('q')):
            break
        elif key == ord('s'):
            out = _params_to_dict(dp)
            with open(args.save, "w") as f:
                yaml.safe_dump(out, f, default_flow_style=False)
            print(f"Saved parameters to {args.save}")
        elif key == ord('n'):
            dict_idx = (dict_idx + 1) % len(available_names)
            cv2.setTrackbarPos("dict_idx", win, dict_idx)
        elif key == ord('p'):
            dict_idx = (dict_idx - 1) % len(available_names)
            cv2.setTrackbarPos("dict_idx", win, dict_idx)

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


