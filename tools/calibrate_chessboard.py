import argparse
import cv2
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description="Calibrate camera from a chessboard video and save npz.")
    parser.add_argument("--video", required=True, help="Path to chessboard video or camera index (e.g., 0).")
    parser.add_argument("--pattern_cols", type=int, default=9, help="Number of inner corners along width.")
    parser.add_argument("--pattern_rows", type=int, default=6, help="Number of inner corners along height.")
    parser.add_argument("--square_size", type=float, default=0.023, help="Square size in meters.")
    parser.add_argument("--skip_frames", type=int, default=10, help="Frames to skip between detections.")
    parser.add_argument("--max_frames", type=int, default=20, help="Stop after collecting this many detections.")
    parser.add_argument("--min_frames", type=int, default=6, help="Minimum detections required to calibrate.")
    parser.add_argument("--output", default="camera_calibration_params.npz", help="Output npz path.")
    parser.add_argument("--no-gui", action="store_true", help="Disable OpenCV windows (headless).")
    return parser.parse_args()


def main():
    args = parse_args()
    pattern_size = (args.pattern_cols, args.pattern_rows)

    # Accept numeric camera index (e.g., "--video 0") or a file path.
    video_source = int(args.video) if args.video.isdigit() else args.video

    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * args.square_size

    object_points = []
    image_points = []

    cap = cv2.VideoCapture(video_source)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {args.video}")

    found_count = 0
    gray = None

    while cap.isOpened():
        for _ in range(args.skip_frames):
            cap.read()
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if ret:
            found_count += 1
            object_points.append(objp)
            refined = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
            image_points.append(refined)
            if not args.no_gui:
                cv2.drawChessboardCorners(frame, pattern_size, refined, ret)
                cv2.imshow("Detected Chessboard", frame)
            if found_count >= args.max_frames:
                break

        if not args.no_gui:
            cv2.imshow("Calibration", frame)
            if cv2.waitKey(2) & 0xFF == ord("q"):
                break

    cap.release()
    if not args.no_gui:
        cv2.destroyAllWindows()

    if found_count >= args.min_frames and gray is not None:
        ret, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
            object_points, image_points, gray.shape[::-1], None, None
        )
        if not ret:
            raise RuntimeError("Calibration failed.")
        np.savez(args.output, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
        print("Calibration successful.")
        print("Saved:", args.output)
        print("Camera Matrix:\n", camera_matrix)
        print("Distortion Coefficients:\n", dist_coeffs)
    else:
        raise RuntimeError("Not enough frames with detected corners to perform calibration.")


if __name__ == "__main__":
    main()
