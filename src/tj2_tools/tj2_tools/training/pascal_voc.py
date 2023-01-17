import os
from typing import Tuple
from lxml import etree
from .training_object import TrainingFrame, TrainingObject


def add_xml_prop(root, name, value=None):
    obj = etree.Element(name)
    if value is not None:
        if type(value) == bool:
            value = int(value)
        value = str(value)
        obj.text = value.encode()

    root.append(obj)
    return obj


def find_xml_prop(root, name):
    element = root.find(name)
    if isinstance(element, etree._Element):
        return element.text
    else:
        return element


class PascalVOCObject(TrainingObject):
    def __init__(self, label, x0, x1, y0, y1, width, height):
        super().__init__(label, x0, x1, y0, y1, width, height)
        self.pose = "Unspecified"
        self.difficult = 0
        self.truncated = 0

    def get_voc_box(self) -> Tuple[float, float, float, float]:
        return (self.bounding_box.x0, self.bounding_box.y0, self.bounding_box.x1, self.bounding_box.y1)

    @classmethod
    def from_obj(cls, obj: TrainingObject):
        self = cls(
            obj.label,
            obj.bounding_box.x0,
            obj.bounding_box.y0,
            obj.bounding_box.x1, 
            obj.bounding_box.y1,
            obj.bounding_box.width,
            obj.bounding_box.height
        )
        if type(obj) == PascalVOCObject:
            self.pose = obj.pose
            self.difficult = obj.difficult
            self.truncated = obj.truncated

        return self

    @classmethod
    def from_element(cls, element, width, height):
        label = find_xml_prop(element, "name")
        bndbox = element.find("bndbox")
        xmin = cls.parse_bndbox_prop(find_xml_prop(bndbox, "xmin"))
        ymin = cls.parse_bndbox_prop(find_xml_prop(bndbox, "ymin"))
        xmax = cls.parse_bndbox_prop(find_xml_prop(bndbox, "xmax"))
        ymax = cls.parse_bndbox_prop(find_xml_prop(bndbox, "ymax"))
        self = cls(label, xmin, xmax, ymin, ymax, width, height)
        self.pose = find_xml_prop(element, "pose")
        self.truncated = self.parse_bndbox_prop(find_xml_prop(element, "truncated"))
        self.difficult = self.parse_bndbox_prop(find_xml_prop(element, "difficult"))

        return self

    @staticmethod
    def parse_bndbox_prop(x):
        try:
            return int(x)
        except ValueError:
            return int(float(x))

    def to_xml(self, root=None):
        if root is None:
            root = etree.Element("object")
        add_xml_prop(root, "name", self.label)
        add_xml_prop(root, "pose", self.pose)
        add_xml_prop(root, "difficult", self.difficult)
        add_xml_prop(root, "truncated", self.truncated)
        bndbox = add_xml_prop(root, "bndbox")
        bounding_box = self.bounding_box.constrain()
        add_xml_prop(bndbox, "xmin", bounding_box.x0)
        add_xml_prop(bndbox, "ymin", bounding_box.y0)
        add_xml_prop(bndbox, "xmax", bounding_box.x1)
        add_xml_prop(bndbox, "ymax", bounding_box.y1)
        return root


class PascalVOCFrame(TrainingFrame):
    def __init__(self, frame_path: str, image_path: str):
        super().__init__(frame_path, image_path)
        self.database = "Unknown"

        self.width = 0
        self.height = 0
        self.depth = 0

        self.segmented = 0
        self.verified = False

    @classmethod
    def from_frame(cls, other: TrainingFrame):
        self = cls(other.frame_path, other.image_path)

        if type(other) == PascalVOCFrame:
            self.database = other.database

            self.width = other.width
            self.height = other.height
            self.depth = other.depth

            self.segmented = other.segmented
            self.verified = other.verified

        return self

    @classmethod
    def from_path(cls, path):
        tree = etree.parse(path)
        verified = tree.getroot().get("verified")
        image_path = find_xml_prop(tree, "path")

        self = cls(path, image_path)
        self.verified = verified == "yes"
        self.database = find_xml_prop(tree, "source/database")
        size = tree.find("size")
        self.width = int(find_xml_prop(size, "width"))
        self.height = int(find_xml_prop(size, "height"))
        self.depth = int(find_xml_prop(size, "depth"))
        self.segmented = int(find_xml_prop(tree, "segmented"))

        for element in tree.getroot().getchildren():
            if element.tag != "object":
                continue
            obj = PascalVOCObject.from_element(element, self.width, self.height)
            self.objects.append(obj)
        return self

    def to_xml(self):
        root = etree.Element("annotation")
        root.set("verified", "yes" if self.verified else "no")
        add_xml_prop(root, "folder", os.path.dirname(self.image_path))
        add_xml_prop(root, "filename", os.path.basename(self.image_path))
        add_xml_prop(root, "path", self.image_path)
        db_src = add_xml_prop(root, "source")
        add_xml_prop(db_src, "database", self.database)

        size = add_xml_prop(root, "size")
        add_xml_prop(size, "width", self.width)
        add_xml_prop(size, "height", self.height)
        add_xml_prop(size, "depth", self.depth)

        add_xml_prop(root, "segmented", self.segmented)

        for obj in self.objects:
            assert type(obj) == PascalVOCObject
            root.append(obj.to_xml())

        return root

    def write(self):
        root = self.to_xml()
        et = etree.ElementTree(root)
        et.write(self.annotation_path, pretty_print=True)
        return root, self.annotation_path

    def __str__(self) -> str:
        return etree.tostring(self.to_xml(), pretty_print=True).decode()
