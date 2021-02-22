import re
import os
import PIL.Image
import shutil
import random

from get_image_size import get_image_metadata, Image
from pascal_voc import PascalVOCFrame

ANNOTATIONS = "Annotations"
IMAGESETS = "ImageSets/Main"
JPEGIMAGES = "JPEGImages"
LABELS = "labels.txt"


def find_labeled_images(source_dir):
    # iterate through FRC team 900's image dataset
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


def generate_label_file(image_metadata: Image, classes, image_path, source_path, database_name):
    source_frame = PascalVOCFrame.from_path(source_path)
    if image_metadata.width != source_frame.width or image_metadata.height != source_frame.height:
        print(f"WARNING: Source xml image size doesn't match actual image. "
              f"Overriding with true image size. "
              f"({image_metadata.width}, {image_metadata.height}) != ({source_frame.width}, {source_frame.height}) "
              f"in {source_path}")
    dest_frame = PascalVOCFrame.from_frame(source_frame)
    dest_frame.width = image_metadata.width
    dest_frame.height = image_metadata.height
    dest_frame.folder = os.path.basename(os.path.dirname(image_path))
    dest_frame.filename = os.path.basename(image_path)
    dest_frame.path = image_path
    dest_frame.database = database_name

    for obj in dest_frame.objects:
        assert obj.name in classes, f"{obj.name} not an available class name: {classes}"

    return dest_frame


def write_labels_file(dest_dir, classes):
    background_class = classes.pop(0)  # remove BACKGROUND class
    assert background_class == "BACKGROUND"
    contents = "\n".join(classes)
    labels_path = os.path.join(dest_dir, LABELS)
    print(f"Labels: {labels_path}")
    with open(labels_path, 'w') as file:
        file.write(contents)


def write_image_as_jpg(old_image_path, new_image_path):
    image = PIL.Image.open(old_image_path)
    rgb_image = image.convert("RGB")
    rgb_image.save(new_image_path)


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
        image_filename = os.path.basename(image_path)
        image_name, image_ext = os.path.splitext(image_filename)
        dest_image_path = os.path.join(jpegimages_dir, image_name + ".jpg")
        image_metadata = get_image_metadata(image_path)
        if not os.path.isfile(dest_image_path):
            if not dry_run:
                if image_ext.lower() == ".jpg":
                    shutil.copyfile(image_path, dest_image_path)
                else:
                    write_image_as_jpg(image_path, dest_image_path)
            print(f"Image: {image_path} -> {dest_image_path}")

        xml_filename = os.path.basename(xml_path)
        frame_id = os.path.splitext(xml_filename)[0]
        dest_xml_path = os.path.join(annotations_dir, xml_filename)
        voc_frame = generate_label_file(image_metadata, classes, dest_image_path, xml_path, database_name)
        dataset[frame_id] = voc_frame
        if not dry_run:
            voc_frame.write(dest_xml_path)
        print(f"Annotation: {xml_path} -> {dest_xml_path}")

    if not dry_run:
        write_labels_file(dest_dir, classes)

    return dataset


def randomly_select(count, input_list, output_list):
    assert count <= len(input_list), f"{count} > {len(input_list)}"

    for _ in range(count):
        obj = random.choice(input_list)
        index = input_list.index(obj)
        output_list.append(input_list.pop(index))


def build_image_sets(database, dest_dir, dry_run=True):
    # given a percentage of test, train, and validation, group all xml-image pairs into one of these categories
    # assign each image a label (the first label that appears in the xml)
    # for each label, randomly select images into each category until there are no unselected images left
    # write the names of these grouping into text files in the destination directory

    # annotations_dir = os.path.join(dest_dir, ANNOTATIONS)
    imagesets_dir = os.path.join(dest_dir, IMAGESETS)
    # jpegimages_dir = os.path.join(dest_dir, JPEGIMAGES)

    train_path = os.path.join(imagesets_dir, "train.txt")
    val_path = os.path.join(imagesets_dir, "val.txt")
    trainval_path = os.path.join(imagesets_dir, "trainval.txt")
    test_path = os.path.join(imagesets_dir, "test.txt")

    test_ratio = 0.15
    train_ratio = 0.8
    validation_ratio = 0.05
    assert abs(1.0 - (test_ratio + train_ratio + validation_ratio)) < 1E-8, test_ratio + train_ratio + validation_ratio

    label_unique_mapping = {}
    for frame_id, frame in database.items():
        # use first object to characterize image for sorting
        if len(frame.objects) == 0:
            class_name = "BACKGROUND"
        else:
            class_name = frame.objects[0].name
        if class_name not in label_unique_mapping:
            label_unique_mapping[class_name] = []
        label_unique_mapping[class_name].append(frame_id)

    test_image_ids = []
    train_image_ids = []
    validation_image_ids = []
    for class_name, frame_ids in label_unique_mapping.items():
        class_count = len(frame_ids)
        test_count = int(class_count * test_ratio)
        train_count = int(class_count * train_ratio)
        validation_count = int(class_count * validation_ratio)

        randomly_select(test_count, frame_ids, test_image_ids)
        randomly_select(train_count, frame_ids, train_image_ids)
        randomly_select(validation_count, frame_ids, validation_image_ids)
        for _ in range(len(frame_ids)):  # dump any remaining frames into the training set
            train_image_ids.append(frame_ids.pop())

        assert len(frame_ids) == 0, len(frame_ids)
        assert len(test_image_ids) > 0
        assert len(train_image_ids) > 0
        assert len(validation_image_ids) > 0

        print(
            f"Label {class_name} count: {class_count}\n"
            f"\tTest: {test_count}\t{len(test_image_ids)}\n"
            f"\tTrain: {train_count}\t{len(train_image_ids)}\n"
            f"\tValidation: {validation_count}\t{len(validation_image_ids)}")

    test_count = len(test_image_ids)
    train_count = len(train_image_ids)
    validation_count = len(validation_image_ids)
    total_count = test_count + train_count + validation_count
    print(
        f"Total {total_count}:\n"
        f"\tTest: {test_count}\t{test_count / total_count:0.2f}\n"
        f"\tTrain: {train_count}\t{train_count / total_count:0.2f}\n"
        f"\tValidation: {validation_count}\t{validation_count / total_count:0.2f}"
    )
    if not dry_run:
        with open(train_path, 'w') as file:
            file.write("\n".join(train_image_ids))
        with open(val_path, 'w') as file:
            file.write("\n".join(validation_image_ids))
        with open(trainval_path, 'w') as file:
            file.write("\n".join(train_image_ids))
            file.write("\n")
            file.write("\n".join(validation_image_ids))
        with open(test_path, 'w') as file:
            file.write("\n".join(test_image_ids))


def makedirs(dest_dir):
    annotations_dir = os.path.join(dest_dir, ANNOTATIONS)
    imagesets_dir = os.path.join(dest_dir, IMAGESETS)
    jpegimages_dir = os.path.join(dest_dir, JPEGIMAGES)
    dirs = [annotations_dir, imagesets_dir, jpegimages_dir]
    for directory in dirs:
        if not os.path.isdir(directory):
            os.makedirs(directory)


def format_frc900_dataset(source_dir, dest_dir, class_label_path, database_name, dry_run=True):
    # find all xmls with an image pair
    # find the class labels
    # move images and xmls to destination directory
    # randomly split the database into test, train, and validation

    dest_dir = os.path.join(dest_dir, database_name)
    makedirs(dest_dir)

    labeled_images = find_labeled_images(source_dir)
    classes = read_class_labels(class_label_path)
    dataset = build_dataset(database_name, labeled_images, classes, dest_dir, dry_run)
    build_image_sets(dataset, dest_dir, dry_run)


def main():
    format_frc900_dataset(
        "/home/ben/tensorflow_workspace/2020Game/data/videos",
        "/home/ben/jetson-inference/python/training/detection/ssd/data",
        "/home/ben/tensorflow_workspace/2020Game/data/2020Game_label_map.pbtxt",
        "tj2_2020_voc_image_database",
        dry_run=False)


if __name__ == '__main__':
    main()
