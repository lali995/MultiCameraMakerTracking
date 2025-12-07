"""
ArUco Marker Generator

Generates printable ArUco markers for tracking.
Supports multiple dictionary types and output formats.
"""

import os
from typing import List, Tuple, Optional
import numpy as np

try:
    import cv2
    from cv2 import aruco
except ImportError:
    raise ImportError("OpenCV with ArUco support required. Install with: pip install opencv-contrib-python")


# ArUco dictionary types
ARUCO_DICT = {
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
}


class MarkerGenerator:
    """Generates printable ArUco markers."""

    def __init__(self, dictionary_type: str = "DICT_4X4_50"):
        """
        Initialize marker generator.

        Args:
            dictionary_type: ArUco dictionary to use. Default is DICT_4X4_50.
                Options: DICT_4X4_50, DICT_4X4_100, DICT_5X5_50, DICT_6X6_50, etc.
        """
        if dictionary_type not in ARUCO_DICT:
            raise ValueError(f"Unknown dictionary type: {dictionary_type}. "
                           f"Available: {list(ARUCO_DICT.keys())}")

        self.dictionary_type = dictionary_type
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[dictionary_type])

    def generate_marker(self, marker_id: int, size_pixels: int = 200,
                       border_bits: int = 1) -> np.ndarray:
        """
        Generate a single ArUco marker image.

        Args:
            marker_id: ID of the marker to generate (must be valid for dictionary)
            size_pixels: Size of the marker in pixels (excluding border)
            border_bits: Width of the white border in bits

        Returns:
            Marker image as numpy array (grayscale)
        """
        # Generate the marker (API differs between OpenCV versions)
        if hasattr(cv2.aruco, 'generateImageMarker'):
            # OpenCV 4.7+
            marker_image = cv2.aruco.generateImageMarker(
                self.aruco_dict, marker_id, size_pixels
            )
        else:
            # OpenCV 4.5.x and earlier
            marker_image = cv2.aruco.drawMarker(
                self.aruco_dict, marker_id, size_pixels
            )

        # Add white border if requested
        if border_bits > 0:
            bit_size = size_pixels // (4 + 2)  # Approximate bit size for 4x4 markers
            border_size = border_bits * bit_size
            marker_with_border = np.ones(
                (size_pixels + 2 * border_size, size_pixels + 2 * border_size),
                dtype=np.uint8
            ) * 255
            marker_with_border[border_size:border_size + size_pixels,
                              border_size:border_size + size_pixels] = marker_image
            marker_image = marker_with_border

        return marker_image

    def save_marker(self, marker_id: int, output_path: str,
                   size_pixels: int = 200, border_bits: int = 1,
                   dpi: int = 300) -> str:
        """
        Generate and save a marker to file.

        Args:
            marker_id: ID of the marker
            output_path: Path to save the marker image
            size_pixels: Size in pixels
            border_bits: White border width
            dpi: DPI for the output image

        Returns:
            Path to the saved file
        """
        marker = self.generate_marker(marker_id, size_pixels, border_bits)

        # Ensure output directory exists
        os.makedirs(os.path.dirname(output_path) if os.path.dirname(output_path) else '.', exist_ok=True)

        cv2.imwrite(output_path, marker)
        return output_path

    def generate_marker_sheet(self, marker_ids: List[int],
                             markers_per_row: int = 4,
                             marker_size_pixels: int = 200,
                             margin_pixels: int = 50,
                             label_markers: bool = True) -> np.ndarray:
        """
        Generate a sheet with multiple markers for printing.

        Args:
            marker_ids: List of marker IDs to include
            markers_per_row: Number of markers per row
            marker_size_pixels: Size of each marker
            margin_pixels: Margin between markers
            label_markers: Whether to add ID labels below markers

        Returns:
            Sheet image as numpy array
        """
        num_markers = len(marker_ids)
        num_rows = (num_markers + markers_per_row - 1) // markers_per_row

        label_height = 30 if label_markers else 0
        cell_width = marker_size_pixels + margin_pixels
        cell_height = marker_size_pixels + margin_pixels + label_height

        sheet_width = markers_per_row * cell_width + margin_pixels
        sheet_height = num_rows * cell_height + margin_pixels

        # Create white sheet
        sheet = np.ones((sheet_height, sheet_width), dtype=np.uint8) * 255

        for idx, marker_id in enumerate(marker_ids):
            row = idx // markers_per_row
            col = idx % markers_per_row

            x = margin_pixels + col * cell_width
            y = margin_pixels + row * cell_height

            # Generate marker without additional border (sheet provides margin)
            marker = self.generate_marker(marker_id, marker_size_pixels, border_bits=0)

            # Place marker on sheet
            sheet[y:y + marker_size_pixels, x:x + marker_size_pixels] = marker

            # Add label if requested
            if label_markers:
                label_y = y + marker_size_pixels + 20
                label_x = x + marker_size_pixels // 2 - 20
                cv2.putText(sheet, f"ID: {marker_id}", (label_x, label_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, 0, 1)

        return sheet

    def save_marker_sheet(self, marker_ids: List[int], output_path: str,
                         markers_per_row: int = 4,
                         marker_size_pixels: int = 200,
                         margin_pixels: int = 50,
                         label_markers: bool = True) -> str:
        """
        Generate and save a marker sheet.

        Args:
            marker_ids: List of marker IDs
            output_path: Path to save the sheet
            markers_per_row: Markers per row
            marker_size_pixels: Size of each marker
            margin_pixels: Margin between markers
            label_markers: Whether to add labels

        Returns:
            Path to saved file
        """
        sheet = self.generate_marker_sheet(
            marker_ids, markers_per_row, marker_size_pixels,
            margin_pixels, label_markers
        )

        os.makedirs(os.path.dirname(output_path) if os.path.dirname(output_path) else '.', exist_ok=True)
        cv2.imwrite(output_path, sheet)
        return output_path

    def generate_marker_with_info(self, marker_id: int,
                                  size_mm: float = 50.0,
                                  dpi: int = 300) -> Tuple[np.ndarray, dict]:
        """
        Generate a marker with print size information.

        Args:
            marker_id: Marker ID
            size_mm: Desired physical size in millimeters
            dpi: Print DPI

        Returns:
            Tuple of (marker_image, info_dict)
        """
        # Calculate pixels for desired physical size at given DPI
        # 1 inch = 25.4 mm
        size_pixels = int((size_mm / 25.4) * dpi)

        marker = self.generate_marker(marker_id, size_pixels, border_bits=1)

        info = {
            'marker_id': marker_id,
            'dictionary': self.dictionary_type,
            'size_mm': size_mm,
            'size_pixels': marker.shape[0],
            'dpi': dpi,
            'print_instructions': f"Print at {dpi} DPI for {size_mm}mm marker size"
        }

        return marker, info

    @staticmethod
    def get_available_dictionaries() -> List[str]:
        """Get list of available ArUco dictionary types."""
        return list(ARUCO_DICT.keys())

    @staticmethod
    def get_dictionary_info(dictionary_type: str) -> dict:
        """
        Get information about a dictionary.

        Args:
            dictionary_type: Dictionary type name

        Returns:
            Dictionary with marker count and bit size info
        """
        info = {
            "DICT_4X4_50": {"bits": "4x4", "count": 50},
            "DICT_4X4_100": {"bits": "4x4", "count": 100},
            "DICT_4X4_250": {"bits": "4x4", "count": 250},
            "DICT_4X4_1000": {"bits": "4x4", "count": 1000},
            "DICT_5X5_50": {"bits": "5x5", "count": 50},
            "DICT_5X5_100": {"bits": "5x5", "count": 100},
            "DICT_5X5_250": {"bits": "5x5", "count": 250},
            "DICT_5X5_1000": {"bits": "5x5", "count": 1000},
            "DICT_6X6_50": {"bits": "6x6", "count": 50},
            "DICT_6X6_100": {"bits": "6x6", "count": 100},
            "DICT_6X6_250": {"bits": "6x6", "count": 250},
            "DICT_6X6_1000": {"bits": "6x6", "count": 1000},
            "DICT_7X7_50": {"bits": "7x7", "count": 50},
            "DICT_7X7_100": {"bits": "7x7", "count": 100},
            "DICT_7X7_250": {"bits": "7x7", "count": 250},
            "DICT_7X7_1000": {"bits": "7x7", "count": 1000},
            "DICT_ARUCO_ORIGINAL": {"bits": "5x5", "count": 1024},
        }
        return info.get(dictionary_type, {"bits": "unknown", "count": 0})


def main():
    """CLI for generating markers."""
    import argparse

    parser = argparse.ArgumentParser(description="Generate printable ArUco markers")
    parser.add_argument("--id", type=int, nargs="+", default=[0, 1, 2, 3],
                       help="Marker ID(s) to generate")
    parser.add_argument("--dict", type=str, default="DICT_4X4_50",
                       choices=list(ARUCO_DICT.keys()),
                       help="ArUco dictionary type")
    parser.add_argument("--size", type=int, default=200,
                       help="Marker size in pixels")
    parser.add_argument("--size-mm", type=float, default=None,
                       help="Physical size in mm (overrides --size)")
    parser.add_argument("--dpi", type=int, default=300,
                       help="DPI for physical size calculation")
    parser.add_argument("--output", "-o", type=str, default="markers",
                       help="Output directory")
    parser.add_argument("--sheet", action="store_true",
                       help="Generate single sheet with all markers")
    parser.add_argument("--per-row", type=int, default=4,
                       help="Markers per row in sheet mode")
    parser.add_argument("--list-dicts", action="store_true",
                       help="List available dictionaries and exit")

    args = parser.parse_args()

    if args.list_dicts:
        print("Available ArUco dictionaries:")
        for name in ARUCO_DICT.keys():
            info = MarkerGenerator.get_dictionary_info(name)
            print(f"  {name}: {info['bits']} bits, {info['count']} markers")
        return

    generator = MarkerGenerator(args.dict)

    # Calculate size
    if args.size_mm:
        size_pixels = int((args.size_mm / 25.4) * args.dpi)
        print(f"Using {size_pixels} pixels for {args.size_mm}mm at {args.dpi} DPI")
    else:
        size_pixels = args.size

    os.makedirs(args.output, exist_ok=True)

    if args.sheet:
        output_path = os.path.join(args.output, f"marker_sheet_{args.dict}.png")
        generator.save_marker_sheet(args.id, output_path,
                                   markers_per_row=args.per_row,
                                   marker_size_pixels=size_pixels)
        print(f"Saved marker sheet: {output_path}")
    else:
        for marker_id in args.id:
            output_path = os.path.join(args.output, f"marker_{args.dict}_{marker_id}.png")
            generator.save_marker(marker_id, output_path, size_pixels)
            print(f"Saved marker {marker_id}: {output_path}")

    print(f"\nDictionary: {args.dict}")
    print(f"Marker size: {size_pixels}px")
    if args.size_mm:
        print(f"Physical size: {args.size_mm}mm at {args.dpi} DPI")


if __name__ == "__main__":
    main()
