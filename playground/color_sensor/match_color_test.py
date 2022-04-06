import math

color_match_table = {
    "red": [1.0, 0.0, 0.0],
    "blue": [0.0, 0.0, 1.0]
}

def match_color(sensed_rgb, confidence_threshold=0.5):
    # sensed_rgb has channels from 0.0..1.0
    magnitude = get_color_distance(sensed_rgb)
    min_dist = 1.0
    match_name = ""
    for name, rgb in color_match_table.items():
        distance = get_color_distance(rgb, sensed_rgb)
        if distance < min_dist:
            min_dist = distance
            match_name = name
    confidence = 1.0 - (min_dist / magnitude)
    if confidence > confidence_threshold:
        return match_name
    else:
        return ""

def get_color_distance(rgb1, rgb2=None):
    if rgb2 is None:
        rgb2 = [0.0, 0.0, 0.0]
    dr = rgb1[0] - rgb2[0]
    dg = rgb1[1] - rgb2[1]
    db = rgb1[2] - rgb2[2]
    return math.sqrt(dr * dr + dg * dg + db * db)



assert match_color([0.8, 0.0, 0.0]) == "red"
assert match_color([0.0, 0.8, 0.0]) == ""
assert match_color([0.0, 0.0, 0.8]) == "blue"
assert match_color([0.9, 0.04, 0.5]) == "red"
