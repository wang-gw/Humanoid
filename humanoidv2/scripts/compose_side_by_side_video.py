#!/usr/bin/env python3
"""Compose two evaluation videos side by side with readable labels."""

from __future__ import annotations

import argparse
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def label_frame(frame: np.ndarray, label: str) -> np.ndarray:
    image = Image.fromarray(frame)
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default(size=22)
    bounds = draw.textbbox((0, 0), label, font=font)
    width = bounds[2] - bounds[0]
    draw.rectangle((8, 8, 24 + width, 42), fill=(0, 0, 0))
    draw.text((16, 13), label, fill=(255, 255, 255), font=font)
    return np.asarray(image)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--left-video", type=Path, required=True)
    parser.add_argument("--right-video", type=Path, required=True)
    parser.add_argument("--left-label", required=True)
    parser.add_argument("--right-label", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    left_reader = imageio.get_reader(args.left_video)
    right_reader = imageio.get_reader(args.right_video)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio.get_writer(args.output, fps=25, macro_block_size=16)
    count = 0
    for left_frame, right_frame in zip(left_reader, right_reader):
        writer.append_data(
            np.concatenate(
                (
                    label_frame(left_frame, args.left_label),
                    label_frame(right_frame, args.right_label),
                ),
                axis=1,
            )
        )
        count += 1
    left_reader.close()
    right_reader.close()
    writer.close()
    print(f"output={args.output} frames={count}")


if __name__ == "__main__":
    main()
