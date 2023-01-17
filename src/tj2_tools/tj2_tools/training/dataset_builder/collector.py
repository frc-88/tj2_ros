from typing import Generator
from ..training_object import TrainingFrame

class Collector:
    def __init__(self):
        pass

    def get_length(self) -> int:
        raise NotImplementedError

    def iter(self) -> Generator[TrainingFrame, None, None]:
        raise NotImplementedError
