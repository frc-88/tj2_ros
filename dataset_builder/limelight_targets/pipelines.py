import numpy as np
from helpers import apply_objects_to_background, \
    blank_background, gauss_background, gauss_foreground, random_color_background, \
    apply_random_shapes

erratic_warps = dict(
    random_width=100,
    lower_ratio=0.8, upper_ratio=1.2,
    random_angle=2 * np.pi,
    min_percentage=1.0, max_percentage=10.0,
    obj_max_count=dict(target_1=3, target_2=4)
)

medium_warps = dict(
    random_width=25,
    lower_ratio=0.8, upper_ratio=1.2,
    random_angle=0.5 * np.pi,
    min_percentage=1.0, max_percentage=10.0,
    obj_max_count=dict(target_1=2, target_2=4)
)

subtle_warps = dict(
    random_width=10,
    lower_ratio=0.2, upper_ratio=10.0,
    random_angle=0.1 * np.pi,
    min_percentage=0.9, max_percentage=10.0,
    obj_max_count=dict(target_1=1, target_2=1)
)

no_warps = dict(
    random_width=0,
    lower_ratio=0.8, upper_ratio=1.2,
    random_angle=0.1 * np.pi,
    min_percentage=1.0, max_percentage=10.0,
    obj_max_count=None
)

background_gauss_params = (20, 20)
foreground_gauss_params = (10, 15)
excluded_hues = list(range(35, 90))
false_shapes_params = (1, 2, 1, 5, ("rect", "circ"), excluded_hues)
false_window_params = (1, 3, 1, 9, ("rect",), list(range(0, 90)) + list(range(120, 256)))


def no_operation(image, frame):
    return image


def with_gauss_background(image, frame):
    background = gauss_background(*background_gauss_params, image.shape)
    return apply_objects_to_background(image, background, frame, no_warps)


def with_gauss(image, frame):
    return gauss_foreground(image, *foreground_gauss_params, image.shape)


def with_colored_background(image, frame):
    background = random_color_background(image.shape, excluded_hues)
    return apply_objects_to_background(image, background, frame, no_warps)


def move_objects_subtle(image, frame):
    background = blank_background(image.shape)
    return apply_objects_to_background(image, background, frame, subtle_warps)


def move_objects_subtle_gauss_background(image, frame):
    background = gauss_background(*background_gauss_params, image.shape)
    return apply_objects_to_background(image, background, frame, subtle_warps)


def move_objects_subtle_gauss(image, frame):
    background = blank_background(image.shape)
    output_image = apply_objects_to_background(image, background, frame, subtle_warps)
    return gauss_foreground(output_image, *foreground_gauss_params, image.shape)


def false_objects_subtle(image, frame):
    background = blank_background(image.shape)
    background = apply_random_shapes(background, *false_shapes_params)
    return apply_objects_to_background(image, background, frame, subtle_warps)


def false_objects_subtle_gauss_background(image, frame):
    background = gauss_background(*background_gauss_params, image.shape)
    background = apply_random_shapes(background, *false_shapes_params)
    return apply_objects_to_background(image, background, frame, subtle_warps)


def false_objects_subtle_gauss(image, frame):
    background = blank_background(image.shape)
    background = apply_random_shapes(background, *false_shapes_params)
    output_image = apply_objects_to_background(image, background, frame, subtle_warps)
    return gauss_foreground(output_image, *foreground_gauss_params, image.shape)


def move_objects_medium(image, frame):
    background = blank_background(image.shape)
    return apply_objects_to_background(image, background, frame, medium_warps)


def move_objects_medium_gauss_background(image, frame):
    background = gauss_background(*background_gauss_params, image.shape)
    return apply_objects_to_background(image, background, frame, medium_warps)


def move_objects_medium_gauss(image, frame):
    background = blank_background(image.shape)
    output_image = apply_objects_to_background(image, background, frame, medium_warps)
    return gauss_foreground(output_image, *foreground_gauss_params, image.shape)


def move_objects_erratic(image, frame):
    background = blank_background(image.shape)
    return apply_objects_to_background(image, background, frame, erratic_warps)


def move_objects_erratic_gauss_background(image, frame):
    background = gauss_background(*background_gauss_params, image.shape)
    return apply_objects_to_background(image, background, frame, erratic_warps)


def move_objects_erratic_gauss(image, frame):
    background = blank_background(image.shape)
    output_image = apply_objects_to_background(image, background, frame, erratic_warps)
    return gauss_foreground(output_image, *foreground_gauss_params, image.shape)


def window_false_objects(image, frame):
    background = blank_background(image.shape)
    background = apply_random_shapes(background, *false_window_params)
    output_image = apply_objects_to_background(image, background, frame, subtle_warps)
    return gauss_foreground(output_image, *foreground_gauss_params, image.shape)
