import argparse
from tj2_northstar.uvc_camera import UVCCamera, CaptureConfig, CameraConfig, Controls


def capture_config_from_args(args):
    return CaptureConfig(
        serial_number=args.serial,
        uid=args.uid,
        product_id=args.product_id,
        vendor_id=args.vendor_id,
        index=args.index,
    )


def list_captures(args) -> None:
    for info in UVCCamera.get_available_captures():
        print(info)


def list_modes(args) -> None:
    config = capture_config_from_args(args)
    capture = UVCCamera("camera", config)
    print(f"Listing modes for {capture.capture}")
    for mode in capture.get_modes():
        print(mode)


def list_controls(args) -> None:
    config = capture_config_from_args(args)
    capture = UVCCamera("camera", config)
    print(f"Listing modes for {capture.capture}")
    for control in capture.get_controls():
        print(control)


def main():
    parser = argparse.ArgumentParser(description="list_uvc")

    sub_parsers = parser.add_subparsers(dest="command", help="")

    sub_parsers.add_parser("captures", help="List available captures")

    modes_parser = sub_parsers.add_parser(
        "modes", help="List available modes for a device"
    )
    modes_parser.add_argument(
        "-s", "--serial", default="", type=str, help="Serial number"
    )
    modes_parser.add_argument("-u", "--uid", default="", type=str, help="UID")
    modes_parser.add_argument(
        "-p", "--product-id", default=0, type=int, help="Product ID"
    )
    modes_parser.add_argument(
        "-v", "--vendor-id", default=0, type=int, help="Vendor ID"
    )
    modes_parser.add_argument(
        "-i",
        "--index",
        default=0,
        type=int,
        help="If there are multiple matches, select this index",
    )

    controls_parser = sub_parsers.add_parser(
        "controls", help="List available controls for a device"
    )
    controls_parser.add_argument(
        "-s", "--serial", default="", type=str, help="Serial number"
    )
    controls_parser.add_argument("-u", "--uid", default="", type=str, help="UID")
    controls_parser.add_argument(
        "-p", "--product-id", default=0, type=int, help="Product ID"
    )
    controls_parser.add_argument(
        "-v", "--vendor-id", default=0, type=int, help="Vendor ID"
    )
    controls_parser.add_argument(
        "-i",
        "--index",
        default=0,
        type=int,
        help="If there are multiple matches, select this index",
    )
    args = parser.parse_args()
    if args.command == "captures":
        list_captures(args)
    elif args.command == "modes":
        list_modes(args)
    elif args.command == "controls":
        list_controls(args)
    else:
        print("Invalid command!")


if __name__ == "__main__":
    main()
