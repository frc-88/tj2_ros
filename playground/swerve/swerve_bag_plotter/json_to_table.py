import os
import csv
from state_loader import get_states


def convert_json(path):
    data = get_states(path)

    out_path = os.path.splitext(path)[0] + ".csv"

    length = min([len(data[module_index]) for module_index in range(len(data))])

    with open(out_path, 'w') as file:
        header = []
        for module_index in range(len(data)):
            for key in data[0][0]:
                header.append("%s_%s" % (key, module_index))
        writer = csv.DictWriter(file, fieldnames=header)
        writer.writeheader()

        for index in range(length):
            row = {}
            for module_index in range(len(data)):
                module_data = data[module_index][index]
                for key in module_data:
                    row["%s_%s" % (key, module_index)] = module_data[key]
            writer.writerow(row)


convert_json("data/diffyjr_2022-06-09-23-55-06_0.json")  # 10.5 V
convert_json("data/diffyjr_2022-06-11-00-38-10_0.json")  # 10.5 V
convert_json("data/diffyjr_2022-06-11-00-44-19_0.json")  # 11.0 V
convert_json("data/diffyjr_2022-06-11-00-46-46_0.json")  # 12.0 V

# no voltage recorded
convert_json("data/diffyjr_2022-06-11-12-54-00_0.json")
convert_json("data/diffyjr_2022-06-11-14-31-53_0.json")
convert_json("data/diffyjr_2022-06-11-14-34-45_0.json")
convert_json("data/diffyjr_2022-06-11-14-39-19_0.json")
convert_json("data/diffyjr_2022-06-11-14-43-10_0.json")
