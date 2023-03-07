# from: https://stackoverflow.com/questions/45531074/how-to-merge-lines-after-houghlinesp

import math
from typing import List

import numpy as np


class HoughBundler:
    @classmethod
    def get_orientation(cls, line: List) -> float:
        """
        Get the angle of a line in degrees.
        line format: [x0, y0, x1, y1]
        """
        orientation = math.atan2((line[3] - line[1]), (line[2] - line[0]))
        return math.degrees(orientation)

    @classmethod
    def _get_matching_group(cls,
        ungrouped_line: List,
        groups: List,
        min_distance_to_merge: float,
        min_angle_to_merge: float,
    ) -> int:
        """
        ungrouped_line format: [x0, y0, x1, y1]
        groups format:
        [
            [  # group 0
                [x0_a, y0_a, x1_a, y1_a],
                [x0_b, y0_b, x1_b, y1_b],
                ...
            ],
            [  # group 1
                [x0_c, y0_c, x1_c, y1_c],
                [x0_d, y0_d, x1_d, y1_d],
                ...
            ],
            ...
        ]

        returns the index in groups ungrouped_line should be appended to
        return -1 if ungrouped_line doesn't match with any group
        """
        for index, group in enumerate(groups):
            for grouped_line in group:
                if cls._get_distance(grouped_line, ungrouped_line) < min_distance_to_merge:
                    orientation_1 = cls.get_orientation(ungrouped_line)
                    orientation_2 = cls.get_orientation(grouped_line)
                    if abs(orientation_1 - orientation_2) < min_angle_to_merge:
                        return index
        return -1

    @classmethod
    def line_magnitude(cls, line: List) -> float:
        """
        Find the length of a line
        format:
        (x0, y0, x1, y1)
        """
        x0, y0, x1, y1 = line
        dx = x1 - x0
        dy = y1 - y0
        return math.sqrt(dx * dx + dy * dy)

    @classmethod
    def _distance_point_to_line(cls, point: List, line: List) -> float:
        """
        Find the shortest distance between point and line
        point format:
        [x, y]

        line format:
        [x0, y0, x1, y1]

        return shortest distance. inf if the line length is < 1e-8
        """
        px, py = point
        x0, y0, x1, y1 = line

        lmag = cls.line_magnitude(line)
        if lmag < 1e-8:
            return float('inf')

        u1 = ((px - x0) * (x1 - x0)) + ((py - y0) * (y1 - y0))
        u = u1 / (lmag * lmag)

        if (u < 0.00001) or (u > 1):
            # closest point does not fall within the line segment, take the shorter distance
            # to an endpoint
            ix = cls.line_magnitude([px, py, x0, y0])
            iy = cls.line_magnitude([px, py, x1, y1])
            if ix > iy:
                distance_point_to_line = iy
            else:
                distance_point_to_line = ix
        else:
            # Intersecting point is on the line, use the formula
            ix = x0 + u * (x1 - x0)
            iy = y0 + u * (y1 - y0)
            distance_point_to_line = cls.line_magnitude([px, py, ix, iy])

        return distance_point_to_line

    @classmethod
    def _get_distance(cls, a_line: List, b_line: List) -> float:
        """
        Find the shortest distance between supplied lines
        a_line format:
        [x0_a, y0_a, x1_a, y1_a]

        b_line format:
        [x0_b, y0_b, x1_b, y1_b]

        return shortest distance
        """
        dist1 = cls._distance_point_to_line(a_line[:2], b_line)
        dist2 = cls._distance_point_to_line(a_line[2:], b_line)
        dist3 = cls._distance_point_to_line(b_line[:2], a_line)
        dist4 = cls._distance_point_to_line(b_line[2:], a_line)

        return min(dist1, dist2, dist3, dist4)

    @classmethod
    def _merge_lines_into_groups(cls, lines: List, min_distance: float, min_angle: float) -> List:
        """
        groups hough lines

        lines format:
        [
            [x0_a, y0_a, x1_a, y1_a],
            [x0_b, y0_b, x1_b, y1_b],
            ...
        ]

        return groups
        groups format:
        [
            [  # group 0
                [x0_a, y0_a, x1_a, y1_a],
                [x0_b, y0_b, x1_b, y1_b],
                ...
            ],
            [  # group 1
                [x0_c, y0_c, x1_c, y1_c],
                [x0_d, y0_d, x1_d, y1_d],
                ...
            ],
            ...
        ]
        """
        groups = []  # all lines groups are here
        # first line will create new group every time
        groups.append([lines[0]])
        # if line is different from existing groups, create a new group
        for line_new in lines[1:]:
            index = cls._get_matching_group(line_new, groups, min_distance, min_angle)
            if index < 0:
                groups.append([line_new])  # create a new group
            else:
                groups[index].append(line_new)

        return groups

    @classmethod
    def _sort_by_index(cls, lines: List, column_index: int) -> None:
        """
        Sort lines by a column

        lines format:
        [
            [x0_a, y0_a, x1_a, y1_a],
            [x0_b, y0_b, x1_b, y1_b],
            ...
        ]
        """
        lines.sort(key=lambda line: line[column_index])  # type: ignore

    @classmethod
    def _merge_line_segments(cls, group: List) -> np.ndarray:
        """
        take all lines in the group and create a new merged line

        group format:
        [
            [x0_a, y0_a, x1_a, y1_a],
            [x0_b, y0_b, x1_b, y1_b],
            ...
        ]

        returns grouped line
        format:
        np.array([x0_c, y0_c, x1_c, y1_c])
        """
        orientation = cls.get_orientation(group[0])

        if len(group) == 1:
            return np.block([[group[0][:2], group[0][2:]]]).flatten()

        points = []
        for line in group:
            points.append(line[:2])
            points.append(line[2:])
        if 45.0 < orientation <= 90.0:
            cls._sort_by_index(points, 1)  # sort by Y
        else:
            cls._sort_by_index(points, 0)  # sort by X

        return np.block([[points[0], points[-1]]]).flatten()

    @classmethod
    def bundle(cls, lines: List, min_distance: float = 5.0, min_angle: float = 2.0) -> np.ndarray:
        """
        take the output of cv2.HoughLinesP and merge lines that
        are likely estimating the same line

        lines format:
        [
            [[x0_a, y0_a, x1_a, y1_a]],
            [[x0_b, y0_b, x1_b, y1_b]],
            ...
        ]

        return format:
        [
            [[x0_c, y0_c, x1_c, y1_c]],
            [[x0_d, y0_d, x1_d, y1_d]],
            ...
        ]
        """
        lines_horizontal = []
        lines_vertical = []

        for line_i in [line[0] for line in lines]:
            orientation = cls.get_orientation(line_i)
            # if vertical
            if 45.0 < orientation <= 90.0:
                lines_vertical.append(line_i)
            else:
                lines_horizontal.append(line_i)

        cls._sort_by_index(lines_vertical, 1)  # sort by Y
        cls._sort_by_index(lines_horizontal, 0)  # sort by X
        merged_lines_all = []

        # for each cluster in vertical and horizantal lines leave only one line
        for sorted_lines in lines_horizontal, lines_vertical:
            if len(sorted_lines) > 0:
                groups = cls._merge_lines_into_groups(sorted_lines, min_distance, min_angle)
                merged_lines = []
                for group in groups:
                    merged_lines.append([cls._merge_line_segments(group)])
                merged_lines_all.extend(merged_lines)

        return np.asarray(merged_lines_all)


if __name__ == '__main__':

    def main() -> None:
        print(HoughBundler.bundle([[[-1.0, -1.0, 1.0, 1.0]], [[2.0, -2.0, -2.0, 2.0]]]))
        print(HoughBundler.bundle([[[-1.0, -1.0, 1.0, 1.0]], [[-2.0, -2.0, 2.0, 2.0]]]))

    main()
