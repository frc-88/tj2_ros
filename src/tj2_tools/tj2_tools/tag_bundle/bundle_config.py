from dataclasses import asdict, dataclass, field
from typing import List

from dacite import from_dict

from tj2_tools.tag_bundle.layout_config import LayoutConfig


@dataclass
class BundleConfig:
    layouts: List[LayoutConfig] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: dict) -> "BundleConfig":
        return from_dict(data_class=cls, data=data)

    def to_dict(self) -> dict:
        return asdict(self)
