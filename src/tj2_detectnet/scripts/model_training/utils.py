import os
import random

from tj2_tools.training.pascal_voc import PascalVOCFrame, PascalVOCObject

ANNOTATIONS = "Annotations"
IMAGESETS = "ImageSets/Main"
JPEGIMAGES = "JPEGImages"
LABELS = "labels.txt"


def list_paths(dir):
    filenames = os.listdir(dir)
    for index, filename in enumerate(filenames):
        filenames[index] = os.path.join(dir, filename)
    return filenames


def read_dataset(source_dir):
    annotations_dir = os.path.join(source_dir, ANNOTATIONS)
    # imagesets_dir = os.path.join(source_dir, IMAGESETS)
    # jpegimages_dir = os.path.join(source_dir, JPEGIMAGES)
    # labels_path = os.path.join(source_dir, LABELS)

    annotation_paths = list_paths(annotations_dir)
    dataset = {}
    for annotation_path in annotation_paths:
        annotation = PascalVOCFrame.from_path(annotation_path)
        dataset[annotation.frame_id] = annotation
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

    # test_ratio = 0.15
    # train_ratio = 0.8
    # validation_ratio = 0.05

    test_ratio = 0.15
    train_ratio = 0.85
    validation_ratio = 0.0
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
        if test_ratio > 0.0:
            assert len(test_image_ids) > 0
        if train_ratio > 0.0:
            assert len(train_image_ids) > 0
        if validation_ratio > 0.0:
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
