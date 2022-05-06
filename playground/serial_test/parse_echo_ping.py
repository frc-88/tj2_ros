
import re
import pyperclip
import numpy as np

s = pyperclip.paste()
raw_data = s.split("---")

data = []
for element in raw_data:
    match = re.search(r"\d+.\d+", element)
    if match is None:
        continue
    datum = float(element[match.start():match.end()])
    data.append(datum)

print("mean:", np.mean(data))
print("stddev:", np.std(data))
print("len:", len(data))
