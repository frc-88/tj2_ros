from typing import List
import numpy as np

from .detection import Detection3d
from .timing_frame import TimingFrame


class TimingReport:
    def __init__(self, num_samples: int) -> None:
        self.num_samples = num_samples
        self.timings: List[TimingFrame] = []

    def timing_report(self, frame: TimingFrame) -> str:
        self.timings.append(frame.copy())

        if len(self.timings) >= 10:
            self.timings.pop(0)

        report = "Callback FPS: %0.2f\n" % (
            1.0 / np.mean(np.diff([frame.start for frame in self.timings]))
        )
        report += "Overall: %0.4fs\n" % (
            np.mean([frame.stop - frame.start for frame in self.timings])
        )
        report += "Resize: %0.4fs\n" % (
            np.mean([frame.resize - frame.start for frame in self.timings])
        )
        report += "Infer: %0.4fs\n" % (
            np.mean([frame.inference - frame.resize for frame in self.timings])
        )
        report += "Detect prep: %0.4fs\n" % (
            np.mean([frame.detect2d_prep - frame.inference for frame in self.timings])
        )
        report += "2D to 3D: %0.4fs\n" % (
            np.mean([frame.to_3d - frame.detect2d_prep for frame in self.timings])
        )

        report += "Get nearest: %0.4fs\n" % (
            np.mean([frame.get_nearest - frame.to_3d for frame in self.timings])
        )
        return report

    def object_report(
        self, detections_3d: List[Detection3d], classes: List[str]
    ) -> str:
        object_counts = {name: 0 for name in classes}
        for detection in detections_3d:
            object_counts[detection.label] += 1
        report = "Object counts:\n"
        for name, count in object_counts.items():
            report += f"\t{name}: {count}\n"
        report += "\n"
        return report
