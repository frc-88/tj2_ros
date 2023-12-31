import os
import argparse
import requests
from PIL import Image

image_formats = {
    "tag16h5": "tag16_05_%05d.png",
    "tag25h9": "tag25_09_%05d.png",
    "tag36h11": "tag36_11_%05d.png",
    "tagCircle21h7": "tag21_07_%05d.png",
    "tagCircle49h12": "tag49_12_%05d.png",
    "tagCustom48h12": "tag48_12_%05d.png",
    "tagStandard41h12": "tag41_12_%05d.png",
    "tagStandard52h13": "tag52_13_%05d.png",
}


def generate_tag_pdf(tag_family: str, code: int, px_per_mm: float, length_mm: float):
    file_name = image_formats[tag_family] % code

    url = f"https://github.com/AprilRobotics/apriltag-imgs/raw/master/{tag_family}/{file_name}"

    img_data = requests.get(url).content
    with open(file_name, "wb") as handler:
        handler.write(img_data)
    print("Downloaded image to", file_name)

    new_border_size_px = int(length_mm * px_per_mm)
    print(f"Resizing to {new_border_size_px} px")

    pdf_path = os.path.splitext(file_name)[0] + ".pdf"

    dpi = px_per_mm * 25.4

    image = Image.open(file_name)

    assert image.size[0] == image.size[1]
    tag_border_px_size = image.size[0]
    tag_px_size = tag_border_px_size - 2
    assert tag_px_size > 0
    resize_ratio = new_border_size_px / tag_border_px_size
    new_tag_px_size = int(tag_px_size * resize_ratio)
    new_tag_size = new_tag_px_size / px_per_mm

    image = image.convert("RGB")
    image = image.resize((new_border_size_px, new_border_size_px), Image.NEAREST)
    image.save(pdf_path, resolution=dpi)
    print(f"PDF path is {pdf_path}")
    print(f"Actual tag size is {new_tag_size} mm (enter this into the apriltag config)")


def main():
    parser = argparse.ArgumentParser(description="tag")

    parser.add_argument("tag", choices=list(image_formats.keys()), help="tag family")
    parser.add_argument(
        "codes", nargs="+", type=int, default=[], help="codes to generate"
    )
    parser.add_argument(
        "-p", "--pixels_per_mm", type=float, default=10.0, help="pixels per mm"
    )
    parser.add_argument(
        "-l", "--length_mm", type=float, default=200.0, help="length mm"
    )

    args = parser.parse_args()

    tag_family = args.tag

    if len(args.codes) == 0:
        print("No codes provided! Nothing to do.")

    for code in args.codes:
        px_per_mm = args.pixels_per_mm
        length_mm = args.length_mm

        generate_tag_pdf(tag_family, code, px_per_mm, length_mm)


if __name__ == "__main__":
    main()

