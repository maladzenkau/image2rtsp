#!/usr/bin/env python3
"""Display an RTSP stream in a window. Press q to quit.

Usage: rtsp.py [URL]     (default: rtsp://127.0.0.1:8554/back)
"""
import argparse
import sys

import cv2


def main():
    parser = argparse.ArgumentParser(description='Display an RTSP stream with OpenCV.')
    parser.add_argument('url', nargs='?', default='rtsp://127.0.0.1:8554/back',
                        help='stream URL (default: %(default)s)')
    args, _ = parser.parse_known_args()  # tolerate --ros-args when started by a launch file

    cap = cv2.VideoCapture(args.url)
    if not cap.isOpened():
        print(f'Could not open {args.url}', file=sys.stderr)
        return 1
    ok, frame = cap.read()
    if not ok:
        print(f'No frames received from {args.url}', file=sys.stderr)
        return 1
    print(f'Connected to {args.url}: {frame.shape[1]}x{frame.shape[0]}, press q to quit', flush=True)

    while ok:
        cv2.imshow('RTSP Stream', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        ok, frame = cap.read()

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
