
def read_class_names(class_names_path):
    with open(class_names_path) as file:
        lines = file.read().splitlines()
    names = []
    for line in lines:
        line = line.strip()
        if len(line) > 0:
            names.append(line)
    return names

