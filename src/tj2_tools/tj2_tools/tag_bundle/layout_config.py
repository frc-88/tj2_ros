from dataclasses import dataclass, field
from typing import List

from tj2_tools.tag_bundle.tag_config import TagConfig


@dataclass
class LayoutConfig:
    layout: List[TagConfig] = field(default_factory=list)
    name: str = ""

    def __post_init__(self) -> None:
        self.layout.sort(key=lambda tag: tag.id)
        self.key = [tag.id for tag in self.layout]
        if len(set(self.key)) != len(self.key):
            raise ValueError("Duplicate tag IDs in layout")
