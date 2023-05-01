from typing import List
from .roi_model.roi_params import classes, IMG_W, IMG_H


class ChargedUpPerceptionParameters:
    def __init__(self) -> None:
        self.classes: List[str] = classes
        self.IMG_W: int = IMG_W
        self.IMG_H: int = IMG_H
