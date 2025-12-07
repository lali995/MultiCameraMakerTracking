#!/usr/bin/env python3
"""
Generate printable ArUco markers.

Usage:
    # Generate individual markers (IDs 0-3)
    python scripts/generate_markers.py

    # Generate a sheet with markers 0-7
    python scripts/generate_markers.py --sheet --id 0 1 2 3 4 5 6 7

    # Generate 50mm markers for printing at 300 DPI
    python scripts/generate_markers.py --size-mm 50 --dpi 300

    # List available dictionaries
    python scripts/generate_markers.py --list-dicts
"""

import os
import sys

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.markers.marker_generator import MarkerGenerator, ARUCO_DICT


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Generate printable ArUco markers",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Generate markers 0-3 as individual files
    python scripts/generate_markers.py --id 0 1 2 3

    # Generate a sheet with all markers
    python scripts/generate_markers.py --sheet --id 0 1 2 3 4 5 6 7

    # Generate 50mm markers for printing
    python scripts/generate_markers.py --size-mm 50 --dpi 300 --id 0 1 2

    # Use different dictionary (more markers available)
    python scripts/generate_markers.py --dict DICT_6X6_250 --id 0 1 2 3
"""
    )

    parser.add_argument("--id", type=int, nargs="+", default=[0, 1, 2, 3],
                       help="Marker ID(s) to generate (default: 0 1 2 3)")
    parser.add_argument("--dict", type=str, default="DICT_4X4_50",
                       choices=list(ARUCO_DICT.keys()),
                       help="ArUco dictionary type (default: DICT_4X4_50)")
    parser.add_argument("--size", type=int, default=200,
                       help="Marker size in pixels (default: 200)")
    parser.add_argument("--size-mm", type=float, default=None,
                       help="Physical size in mm (overrides --size)")
    parser.add_argument("--dpi", type=int, default=300,
                       help="DPI for physical size calculation (default: 300)")
    parser.add_argument("--output", "-o", type=str, default="markers",
                       help="Output directory (default: markers)")
    parser.add_argument("--sheet", action="store_true",
                       help="Generate single sheet with all markers")
    parser.add_argument("--per-row", type=int, default=4,
                       help="Markers per row in sheet mode (default: 4)")
    parser.add_argument("--list-dicts", action="store_true",
                       help="List available dictionaries and exit")

    args = parser.parse_args()

    if args.list_dicts:
        print("Available ArUco dictionaries:\n")
        for name in ARUCO_DICT.keys():
            info = MarkerGenerator.get_dictionary_info(name)
            print(f"  {name}")
            print(f"    - Bit size: {info['bits']}")
            print(f"    - Max markers: {info['count']}")
            print()
        print("Recommended: DICT_4X4_50 for simple setups (fast detection)")
        print("             DICT_6X6_250 for larger marker counts")
        return

    # Validate marker IDs
    dict_info = MarkerGenerator.get_dictionary_info(args.dict)
    max_id = dict_info['count'] - 1
    for marker_id in args.id:
        if marker_id < 0 or marker_id > max_id:
            print(f"Error: Marker ID {marker_id} is out of range for {args.dict}")
            print(f"       Valid range: 0-{max_id}")
            sys.exit(1)

    generator = MarkerGenerator(args.dict)

    # Calculate size
    if args.size_mm:
        size_pixels = int((args.size_mm / 25.4) * args.dpi)
        print(f"Generating {args.size_mm}mm markers at {args.dpi} DPI ({size_pixels} pixels)")
    else:
        size_pixels = args.size
        print(f"Generating {size_pixels}px markers")

    os.makedirs(args.output, exist_ok=True)

    if args.sheet:
        output_path = os.path.join(args.output, f"marker_sheet_{args.dict}.png")
        generator.save_marker_sheet(
            args.id, output_path,
            markers_per_row=args.per_row,
            marker_size_pixels=size_pixels
        )
        print(f"\nSaved marker sheet: {output_path}")
        print(f"  - Contains {len(args.id)} markers: {args.id}")
    else:
        print(f"\nGenerating {len(args.id)} individual markers...")
        for marker_id in args.id:
            output_path = os.path.join(args.output, f"marker_{args.dict}_{marker_id}.png")
            generator.save_marker(marker_id, output_path, size_pixels)
            print(f"  - Saved marker {marker_id}: {output_path}")

    print(f"\nDictionary: {args.dict}")
    print(f"Output directory: {args.output}")

    if args.size_mm:
        print(f"\nPrint instructions:")
        print(f"  1. Print at exactly {args.dpi} DPI")
        print(f"  2. Each marker will be {args.size_mm}mm x {args.size_mm}mm")
        print(f"  3. Use this marker size ({args.size_mm / 1000:.4f}m) in the tracker config")


if __name__ == "__main__":
    main()
