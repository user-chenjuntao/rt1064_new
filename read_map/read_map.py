"""Extract coordinates of map elements from a text grid.

Elements:
  #: obstacle
  @ or C: car
  $ or B: box
  T (or . when no C is present): target
  *: bomb

Coordinates are reported as {row,column} using zero-based indices.
"""

import argparse
from pathlib import Path
from typing import Dict, List, Tuple


SYMBOLS = ["#", "@", "$", ".", "*"]
# Map alternate symbols to canonical ones so the output keys stay stable.
ALIASES = {"C": "@", "B": "$", "T": "."}


def parse_map(path: Path) -> Dict[str, List[Tuple[int, int]]]:
    """Return coordinates of all tracked symbols keyed by the symbol."""
    lines = path.read_text().splitlines()
    car_with_c = any("C" in line for line in lines)
    coords: Dict[str, List[Tuple[int, int]]] = {s: [] for s in SYMBOLS}

    for row, line in enumerate(lines):
        for col, ch in enumerate(line):
            if car_with_c and ch == ".":
                # When 'C' is used for the car, '.' is just floor.
                continue
            symbol = ALIASES.get(ch, ch)
            if symbol in coords:
                coords[symbol].append((row, col))

    return coords


def format_coords(coords: Dict[str, List[Tuple[int, int]]]) -> str:
    """Format coordinates as '{row,col}' lists grouped by symbol."""
    lines = []
    for symbol in SYMBOLS:
        pairs = ", ".join(f"{{{row},{col}}}" for row, col in coords[symbol])
        lines.append(f"{symbol}: {pairs}")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Read a map file and output coordinates for # @ $ . * symbols."
    )
    base_dir = Path(__file__).parent
    parser.add_argument(
        "map_file",
        nargs="?",
        type=Path,
        default=base_dir / "map.txt",
        help="Path to the map text file (default: map.txt beside this script).",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=base_dir / "map_coords.txt",
        help="Path to write the formatted coordinates (default: map_coords.txt beside this script).",
    )
    parser.add_argument(
        "--stdout",
        action="store_true",
        help="Print to stdout instead of writing the output file.",
    )
    args = parser.parse_args()

    coords = parse_map(args.map_file)
    text = format_coords(coords)

    if args.stdout:
        print(text)
        return

    args.output.write_text(text, encoding="utf-8")
    print(f"Wrote coordinates to {args.output}")


if __name__ == "__main__":
    main()