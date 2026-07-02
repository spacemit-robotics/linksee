#!/usr/bin/env python3
"""
Generate a printable ChArUco board for hand-eye calibration.

Defaults match calibrate_hand_eye.py:
  5x7 squares, 25mm square length, 18mm ArUco marker length, 4x4_50 dictionary.
"""

import argparse
from pathlib import Path

import cv2


ARUCO_DICTS = {
    "4x4_50": cv2.aruco.DICT_4X4_50,
    "4x4_100": cv2.aruco.DICT_4X4_100,
    "5x5_50": cv2.aruco.DICT_5X5_50,
    "5x5_100": cv2.aruco.DICT_5X5_100,
    "6x6_50": cv2.aruco.DICT_6X6_50,
    "6x6_100": cv2.aruco.DICT_6X6_100,
}


def create_board(squares_x, squares_y, square_length, marker_length, dictionary):
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dictionary])
    try:
        return cv2.aruco.CharucoBoard(
            (squares_x, squares_y), square_length, marker_length, aruco_dict)
    except TypeError:
        return cv2.aruco.CharucoBoard_create(
            squares_x, squares_y, square_length, marker_length, aruco_dict)


def draw_board(board, width_px, height_px, margin_px, border_bits):
    size = (width_px, height_px)
    if hasattr(board, "generateImage"):
        return board.generateImage(size, marginSize=margin_px, borderBits=border_bits)
    return board.draw(size, marginSize=margin_px, borderBits=border_bits)


def main():
    ap = argparse.ArgumentParser(description="Generate a printable ChArUco board PNG.")
    ap.add_argument("--squares-x", type=int, default=5)
    ap.add_argument("--squares-y", type=int, default=7)
    ap.add_argument("--square-length", type=float, default=0.025,
                    help="Square length in meters. Default: 0.025")
    ap.add_argument("--marker-length", type=float, default=0.018,
                    help="Marker length in meters. Default: 0.018")
    ap.add_argument("--dictionary", choices=sorted(ARUCO_DICTS.keys()),
                    default="4x4_50")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--margin-mm", type=float, default=8.0)
    ap.add_argument("--output", type=str, default=None)
    args = ap.parse_args()

    if args.marker_length >= args.square_length:
        raise SystemExit("--marker-length must be smaller than --square-length")

    width_m = args.squares_x * args.square_length
    height_m = args.squares_y * args.square_length
    px_per_m = args.dpi / 0.0254
    width_px = int(round(width_m * px_per_m))
    height_px = int(round(height_m * px_per_m))
    margin_px = int(round((args.margin_mm / 1000.0) * px_per_m))

    board = create_board(args.squares_x, args.squares_y,
                         args.square_length, args.marker_length,
                         args.dictionary)
    image = draw_board(board, width_px, height_px, margin_px, border_bits=1)

    if args.output is None:
        script_dir = Path(__file__).resolve().parent
        out_dir = script_dir / ".." / "resources"
        sq_mm = int(round(args.square_length * 1000))
        mk_mm = int(round(args.marker_length * 1000))
        args.output = str(
            out_dir / f"charuco_{args.squares_x}x{args.squares_y}_"
            f"{sq_mm}mm_{mk_mm}mm_{args.dictionary}.png")

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), image)

    print(f"Generated: {out_path}")
    print(f"Board: {args.squares_x}x{args.squares_y}, dictionary={args.dictionary}")
    print(f"square_length={args.square_length:.4f}m, marker_length={args.marker_length:.4f}m")
    print("Print at 100% scale, then measure one square and use that value for --charuco-square-length.")


if __name__ == "__main__":
    main()
