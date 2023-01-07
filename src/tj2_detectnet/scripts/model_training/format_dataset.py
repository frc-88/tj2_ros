import re
import os
import PIL.Image
import cv2
import shutil
import random

from tj2_tools.training.get_image_size import get_image_metadata, Image
from tj2_tools.training.pascal_voc import PascalVOCFrame, PascalVOCObject
from utils import ANNOTATIONS, IMAGESETS, JPEGIMAGES, LABELS
from utils import makedirs, build_image_sets


def find_labeled_images(source_dir):
    # iterate through image dataset
    # find all VOC xml files
    # find corresponding images

    labeled_images = []  # [xml path, image path]
    xml_paths = []
    xml_names = []
    image_paths = []
    image_names = []
    for dirpath, dirnames, filenames in os.walk(source_dir):
        for filename in filenames:
            path = os.path.join(dirpath, filename)
            name, ext = os.path.splitext(filename)
            ext = ext.lower()
            if ext == ".xml":
                xml_paths.append(path)
                xml_names.append(name)
            elif ext == ".png" or ext == ".jpg":
                image_paths.append(path)
                image_names.append(name)
    assert len(xml_names) > 0
    for xml_index, name in enumerate(xml_names):
        try:
            image_index = image_names.index(name)
        except ValueError:
            print("WARNING: %s has no corresponding image" % name)
            continue
        xml_path = xml_paths[xml_index]
        image_path = image_paths[image_index]
        assert name in xml_path
        assert name in image_path
        labeled_images.append((xml_path, image_path))
        # print(xml_path, image_path)

    return labeled_images


def read_class_labels(path):
    # read class labels file
    with open(path) as file:
        contents = file.read()
    class_map = {0: "BACKGROUND"}
    max_id = 0

    matches = re.findall("id: (.*)[.\s]*name: '(.*)'", contents, re.MULTILINE)
    for match in matches:
        id_number = int(match[0])
        label_name = match[1]
        class_map[id_number] = label_name
        if id_number > max_id:
            max_id = id_number

    classes = []
    for id_number in range(max_id + 1):
        if id_number not in class_map:
            print("WARNING: id numbers don't ascend in consecutive order. %s is not in source file" % id_number)
        label_name = class_map[id_number]
        classes.append(label_name)

    return classes


def generate_label_file(image_metadata: Image, classes, image_path, source_path, database_name, resize_ratios):
    source_frame = PascalVOCFrame.from_path(source_path)
    if image_metadata.width != source_frame.width or image_metadata.height != source_frame.height:
        print(f"WARNING: Source xml image size doesn't match actual image. "
              f"Overriding with true image size. "
              f"({image_metadata.width}, {image_metadata.height}) != ({source_frame.width}, {source_frame.height}) "
              f"in {source_path}")
    width = image_metadata.width
    height = image_metadata.height

    dest_frame = PascalVOCFrame.from_frame(source_frame)
    dest_frame.width = width
    dest_frame.height = height
    dest_frame.folder = os.path.basename(os.path.dirname(image_path))
    dest_frame.filename = os.path.basename(image_path)
    dest_frame.path = image_path
    dest_frame.database = database_name

    dest_frame.objects = []
    for obj in source_frame.objects:
        # TODO: remove special case for incorrect labels
        if obj.name == "powercell":
            obj.name = "power_cell"
        if obj.name != "power_cell":
            continue
        # assert obj.name in classes, f"{obj.name} not an available class name: {classes}"
        new_obj = PascalVOCObject.from_obj(obj)
        new_obj.bndbox = [
            int(obj.bndbox[0] * resize_ratios[0]),
            int(obj.bndbox[1] * resize_ratios[1]),
            int(obj.bndbox[2] * resize_ratios[0]),
            int(obj.bndbox[3] * resize_ratios[1]),
        ]
        if new_obj.is_out_of_bounds(width, height):
            continue
        new_obj.truncated = new_obj.is_truncated(width, height)
        new_obj.constrain_bndbox(width, height)
        dest_frame.add_object(new_obj)

    return dest_frame


def write_labels_file(dest_dir, classes):
    background_class = classes.pop(0)  # remove BACKGROUND class
    assert background_class == "BACKGROUND"
    contents = "\n".join(classes)
    labels_path = os.path.join(dest_dir, LABELS)
    print(f"Labels: {labels_path}")
    with open(labels_path, 'w') as file:
        file.write(contents)


MAX_WIDTH = 1280
MAX_HEIGHT = 720


def get_resize_ratio(image_metadata):
    width = image_metadata.width
    height = image_metadata.height
    size_is_ok = width <= MAX_WIDTH and height <= MAX_HEIGHT

    if not size_is_ok:
        ratio = min(MAX_WIDTH / width, MAX_HEIGHT / height)
    else:
        ratio = 1.0

    return ratio


def write_image_as_jpg(image_metadata, old_image_path, new_image_path, resize_ratio, dry_run=True):
    width = image_metadata.width
    height = image_metadata.height
    ext_is_ok = os.path.splitext(old_image_path)[1].lower() == ".jpg"
    file_exists = os.path.isfile(new_image_path)

    if ext_is_ok and resize_ratio == 1.0:
        if not dry_run and not file_exists:
            shutil.copyfile(old_image_path, new_image_path)
    else:
        if width > MAX_WIDTH or height > MAX_HEIGHT:
            new_size = int(width * resize_ratio), int(height * resize_ratio)
            print(f"Resizing {new_image_path} from {(width, height)} to {new_size}")
        else:
            new_size = None

        if not dry_run and not file_exists:
            image = cv2.imread(old_image_path)
            if new_size is not None:
                image = cv2.resize(image, new_size)
            cv2.imwrite(new_image_path, image)
            # image = PIL.Image.open(old_image_path)
            # rgb_image = image.convert("RGB")
            # if new_size is not None:
            #     rgb_image = rgb_image.resize(new_size, PIL.Image.ANTIALIAS)
            # rgb_image.save(new_image_path)


def write_image_xml_pair(dataset, database_name, classes, jpegimages_dir, annotations_dir, image_path, xml_path,
                         dry_run=True):
    image_filename = os.path.basename(image_path)
    image_name, image_ext = os.path.splitext(image_filename)
    dest_image_path = os.path.join(jpegimages_dir, image_name + ".jpg")
    image_metadata = get_image_metadata(image_path)
    resize_ratio = get_resize_ratio(image_metadata)

    xml_filename = os.path.basename(xml_path)
    frame_id = os.path.splitext(xml_filename)[0]
    dest_xml_path = os.path.join(annotations_dir, xml_filename)
    voc_frame = generate_label_file(image_metadata, classes, dest_image_path, xml_path, database_name,
                                    (resize_ratio, resize_ratio))
    if len(voc_frame.objects) == 0:
        print(f"No class labels from list found in {xml_path}")
        return

    dataset[frame_id] = voc_frame
    if not dry_run:
        voc_frame.write(dest_xml_path)
    print(f"Annotation: {xml_path} -> {dest_xml_path}")

    write_image_as_jpg(image_metadata, image_path, dest_image_path, resize_ratio, dry_run)
    print(f"Image: {image_path} -> {dest_image_path}")


def build_dataset(database_name, labeled_images, classes, dest_dir, dry_run=True):
    """
    dataset structure:
    Annotations - xml file containing label information and location of image
    ImageSets - text files containing names of label files that should be in the train, test, or validation phases
    JPEGImages - all image files that Annotations point to (must be image type JPEG)

    for each xml-image pair,
      if image is already in the destination directory, skip this image
      get image size
      move copy of image to destination directory

      parse xml file
      set folder, filename, and path according to dest_dir
      set database to database_name
      set verified flag accordingly
      set size tag to image size
      for each object label
          verify label name is in the class labels list
          add bounding box and other fields to VOC object
      set object labels to list of VOC objects
      write xml file to destination directory
    """

    annotations_dir = os.path.join(dest_dir, ANNOTATIONS)
    # imagesets_dir = os.path.join(dest_dir, IMAGESETS)
    jpegimages_dir = os.path.join(dest_dir, JPEGIMAGES)

    dataset = {}
    for xml_path, image_path in labeled_images:
        write_image_xml_pair(dataset, database_name, classes,
                             jpegimages_dir, annotations_dir,
                             image_path, xml_path, dry_run)

    if not dry_run:
        write_labels_file(dest_dir, classes)

    return dataset


def format_dataset(source_dirs, dest_dir, class_label_path, database_name, dry_run=True):
    # find all xmls with an image pair
    # find the class labels
    # move images and xmls to destination directory
    # randomly split the database into test, train, and validation

    dest_dir = os.path.join(dest_dir, database_name)
    makedirs(dest_dir)

    labeled_images = []
    for source_dir in source_dirs:
        labeled_images.extend(find_labeled_images(source_dir))
    classes = read_class_labels(class_label_path)
    dataset = build_dataset(database_name, labeled_images, classes, dest_dir, dry_run)
    build_image_sets(dataset, dest_dir, dry_run)


def main():
    source_dir = "/home/ben/object-recognition/jetson_inference_training/detection/ssd/data"
    ws_dir = source_dir + "/tj2_2020_unsorted"
    format_dataset(
        [
            ws_dir + "/team900_2020_game",
            ws_dir + "/tj2_02-27-2021_2",
            ws_dir + "/realsense_2021-10-23-18-05-29"
         ],
        source_dir,
        ws_dir + "/labels.pbtxt",
        "tj2_2020_voc_image_database",
        dry_run=False)


if __name__ == '__main__':
    main()
