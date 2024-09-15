import csv
import re


def convert_to_list(entry):
    parts = entry.split("], [")

    first_part = parts[0].replace("[", "").replace("]", "").split(", ")

    quaternion_str = parts[1].replace("]", "").replace("[", "")
    quaternion_values = re.findall(r"[-+]?\d*\.\d+|\d+", quaternion_str)

    first_list = [float(value) for value in first_part]
    quaternion_list = [float(value) for value in quaternion_values]

    return first_list + quaternion_list


input_file = "path/to/input.csv"
output_data = []

with open(input_file, "r") as file:
    reader = csv.reader(file)
    for row in reader:
        if row[1] == "0":
            print(row)
            continue
        entry = row[1]
        print(row)
        converted_entry = convert_to_list(entry)
        output_data.append(converted_entry)

for entry in output_data:
    print(entry)

output_file = "path/to/output.csv"
with open(output_file, "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["coord_x", "coord_y", "coord_z", "w", "x", "y", "z"])
    writer.writerows(output_data)
