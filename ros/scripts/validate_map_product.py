#!/usr/bin/env python3

import argparse
import math
from pathlib import Path
import sys


def data_line_count(path: Path, allow_header: bool = False) -> int:
    lines = [line.strip() for line in path.read_text().splitlines() if line.strip()]
    if allow_header and lines and lines[0].startswith("#"):
        lines = lines[1:]
    return len(lines)


def validate_numeric_file(path: Path, allow_header: bool = False) -> None:
    for number, line in enumerate(path.read_text().splitlines(), start=1):
        line = line.strip()
        if not line or (allow_header and line.startswith("#")):
            continue
        for token in line.split():
            try:
                value = float(token)
            except ValueError as exc:
                raise ValueError(f"{path}:{number}: non-numeric token {token!r}") from exc
            if not math.isfinite(value):
                raise ValueError(f"{path}:{number}: non-finite value {token!r}")


def pcd_point_count(path: Path) -> int:
    with path.open("rb") as stream:
        for raw_line in stream:
            line = raw_line.decode("ascii", errors="strict").strip()
            if line.startswith("POINTS "):
                return int(line.split()[1])
            if line.startswith("DATA "):
                break
    raise ValueError(f"{path}: missing PCD POINTS header")


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate a KISS-SAM map data product")
    parser.add_argument("product", type=Path)
    args = parser.parse_args()
    product = args.product.resolve()

    required = [
        product / "metadata.yaml",
        product / "timestamps.txt",
        product / "odometry_tum.txt",
        product / "optimized_trajectory_tum.txt",
        product / "keyframes",
    ]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise FileNotFoundError("missing required products: " + ", ".join(missing))

    timestamps = product / "timestamps.txt"
    odometry = product / "odometry_tum.txt"
    optimized = product / "optimized_trajectory_tum.txt"
    validate_numeric_file(timestamps)
    validate_numeric_file(odometry)
    validate_numeric_file(optimized, allow_header=True)

    timestamp_count = data_line_count(timestamps)
    odometry_count = data_line_count(odometry)
    optimized_count = data_line_count(optimized, allow_header=True)
    keyframes = sorted((product / "keyframes").glob("*.pcd"))
    if len({timestamp_count, odometry_count, optimized_count, len(keyframes)}) != 1:
        raise ValueError(
            "inconsistent counts: "
            f"timestamps={timestamp_count}, odometry={odometry_count}, "
            f"optimized={optimized_count}, keyframes={len(keyframes)}"
        )

    previous_stamp = -math.inf
    for number, line in enumerate(timestamps.read_text().splitlines(), start=1):
        if not line.strip():
            continue
        stamp = float(line.split()[1])
        if stamp <= previous_stamp:
            raise ValueError(f"{timestamps}:{number}: timestamp is not strictly increasing")
        previous_stamp = stamp

    for keyframe in keyframes:
        if pcd_point_count(keyframe) <= 0:
            raise ValueError(f"{keyframe}: empty keyframe")

    maps = sorted(product.glob("*_map.pcd"))
    if len(maps) != 1 or pcd_point_count(maps[0]) <= 0:
        raise ValueError("expected exactly one non-empty final *_map.pcd")

    metadata = (product / "metadata.yaml").read_text()
    if "format_version: 1" not in metadata:
        raise ValueError("unsupported or missing metadata format_version")

    print(
        f"OK: {product} contains {len(keyframes)} keyframes and "
        f"{pcd_point_count(maps[0])} final-map points"
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:  # Keep CLI failure concise for CI and operators.
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
