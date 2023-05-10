import csv
import time
import warnings
from typing import Dict, Union, ByteString, Callable, Tuple, List
from networktables import NetworkTable, NetworkTablesInstance, NetworkTableEntry

try:
    import tqdm
except ImportError:
    tqdm = None

ConvertedNtValue = Union[
    float, str, bool, bytes, List[float], List[str], List[bool], List[bytes]
]
TableBackup = Dict[str, Tuple[str, ConvertedNtValue]]


NT_TYPE_GET_MAPPING: Dict[
    ByteString, Callable[[NetworkTableEntry], ConvertedNtValue]
] = {
    NetworkTablesInstance.EntryTypes.BOOLEAN: lambda entry: entry.getBoolean(False),
    NetworkTablesInstance.EntryTypes.DOUBLE: lambda entry: entry.getDouble(0.0),
    NetworkTablesInstance.EntryTypes.STRING: lambda entry: entry.getString(""),
    NetworkTablesInstance.EntryTypes.RAW: lambda entry: entry.getRaw(b""),
    NetworkTablesInstance.EntryTypes.BOOLEAN_ARRAY: lambda entry: [
        bool(x) for x in entry.getBooleanArray([])
    ],
    NetworkTablesInstance.EntryTypes.DOUBLE_ARRAY: lambda entry: [
        float(x) for x in entry.getDoubleArray([])
    ],
    NetworkTablesInstance.EntryTypes.STRING_ARRAY: lambda entry: [
        str(x) for x in entry.getStringArray([])
    ],
}


def setBoolean(entry: NetworkTableEntry, value: ConvertedNtValue) -> bool:
    assert type(value) == bool
    return entry.setBoolean(value)


def setDouble(entry: NetworkTableEntry, value: ConvertedNtValue) -> bool:
    assert type(value) == float
    return entry.setDouble(value)


def setString(entry: NetworkTableEntry, value: ConvertedNtValue) -> bool:
    assert type(value) == str
    return entry.setString(value)


def setRaw(entry: NetworkTableEntry, value: ConvertedNtValue) -> bool:
    assert type(value) == bytes
    return entry.setRaw(value)


def setBooleanArray(entry: NetworkTableEntry, value: ConvertedNtValue) -> bool:
    assert type(value) == list
    return entry.setBooleanArray(value)  # type: ignore


def setDoubleArray(entry: NetworkTableEntry, value: ConvertedNtValue) -> bool:
    assert type(value) == list
    return entry.setDoubleArray(value)  # type: ignore


def setStringArray(entry: NetworkTableEntry, value: ConvertedNtValue) -> bool:
    assert type(value) == list
    return entry.setStringArray(value)  # type: ignore


NT_TYPE_SET_MAPPING: Dict[
    ByteString, Callable[[NetworkTableEntry, ConvertedNtValue], bool]
] = {
    NetworkTablesInstance.EntryTypes.BOOLEAN: setBoolean,
    NetworkTablesInstance.EntryTypes.DOUBLE: setDouble,
    NetworkTablesInstance.EntryTypes.STRING: setString,
    NetworkTablesInstance.EntryTypes.RAW: setRaw,
    NetworkTablesInstance.EntryTypes.BOOLEAN_ARRAY: setBooleanArray,
    NetworkTablesInstance.EntryTypes.DOUBLE_ARRAY: setDoubleArray,
    NetworkTablesInstance.EntryTypes.STRING_ARRAY: setStringArray,
}

NT_TYPE_NAME_MAPPING = {
    NetworkTablesInstance.EntryTypes.BOOLEAN: "bool",
    NetworkTablesInstance.EntryTypes.DOUBLE: "float",
    NetworkTablesInstance.EntryTypes.STRING: "str",
    NetworkTablesInstance.EntryTypes.RAW: "bytes",
    NetworkTablesInstance.EntryTypes.BOOLEAN_ARRAY: "List[bool]",
    NetworkTablesInstance.EntryTypes.DOUBLE_ARRAY: "List[float]",
    NetworkTablesInstance.EntryTypes.STRING_ARRAY: "List[str]",
}
NT_TYPE_NAME_REVERSE_MAPPING = {v: k for k, v in NT_TYPE_NAME_MAPPING.items()}
NT_TYPE_NAME_DESERIALIZE_MAPPING: Dict[
    ByteString, Callable[[List[str]], ConvertedNtValue]
] = {
    NetworkTablesInstance.EntryTypes.BOOLEAN: lambda val: [bool(x) for x in val],
    NetworkTablesInstance.EntryTypes.DOUBLE: lambda val: [float(x) for x in val],
    NetworkTablesInstance.EntryTypes.STRING: lambda val: [str(x) for x in val],
    NetworkTablesInstance.EntryTypes.RAW: lambda val: [x.encode() for x in val],
    NetworkTablesInstance.EntryTypes.BOOLEAN_ARRAY: lambda val: [bool(x) for x in val],
    NetworkTablesInstance.EntryTypes.DOUBLE_ARRAY: lambda val: [float(x) for x in val],
    NetworkTablesInstance.EntryTypes.STRING_ARRAY: lambda val: [str(x) for x in val],
}
NT_ARRAY_TYPES = [
    NetworkTablesInstance.EntryTypes.BOOLEAN_ARRAY,
    NetworkTablesInstance.EntryTypes.DOUBLE_ARRAY,
    NetworkTablesInstance.EntryTypes.STRING_ARRAY,
]


def recurse_nt(data: TableBackup, current_path: str, table: NetworkTable):
    for entry_name in table.getKeys():
        entry = table.getEntry(entry_name)
        entry_type = entry.getType()
        get_fn = NT_TYPE_GET_MAPPING[entry_type]
        value = get_fn(table.getEntry(entry_name))
        data[current_path + "/" + entry_name] = (
            NT_TYPE_NAME_MAPPING[entry_type],
            value,
        )
    for key in table.getSubTables():
        next_path = current_path + "/" + key
        recurse_nt(data, next_path, table.getSubTable(key))


class Backups:
    @classmethod
    def get_full_table(cls, root_table: NetworkTable) -> TableBackup:
        table: TableBackup = {}
        path = root_table.getPath()
        if path.endswith("/"):
            path = path[:-1]
        recurse_nt(table, path, root_table)
        return table

    @classmethod
    def write_full_table(
        cls, root_table: NetworkTable, table: TableBackup, write_delay: float = 0.0
    ) -> bool:
        pbar = None if tqdm is None else tqdm.tqdm(total=len(table))
        for path, (nt_type, value) in table.items():
            nt_type_code = NT_TYPE_NAME_REVERSE_MAPPING[nt_type]
            set_fn = NT_TYPE_SET_MAPPING[nt_type_code]
            if not set_fn(root_table.getEntry(path), value):
                warnings.warn(f"Failed to set {path} with value {value}")
                return False
            if pbar is not None:
                pbar.update(1)
            if write_delay > 0.0:
                time.sleep(write_delay)
        return True

    @classmethod
    def write_backup(cls, path: str, table: TableBackup) -> None:
        with open(path, "w") as file:
            writer = csv.writer(file)
            for nt_path, (nt_type, value) in table.items():
                row: List[ConvertedNtValue] = [nt_path, nt_type]
                if type(value) == list:
                    row.append(len(value))
                    row.extend(value)
                else:
                    row.append(1)
                    row.append(value)
                writer.writerow(row)

    @classmethod
    def read_backup(cls, path: str) -> TableBackup:
        table: TableBackup = {}
        with open(path) as file:
            reader = csv.reader(file)
            for index, row in enumerate(reader):
                nt_path = row[0]
                nt_type = row[1]
                length = int(row[2])
                raw_values = row[3 : 3 + length]
                if nt_type not in NT_TYPE_NAME_REVERSE_MAPPING:
                    warnings.warn(
                        f"Found invalid NT type '{nt_type}' in {path} on row {index}"
                    )
                    continue
                nt_type_code = NT_TYPE_NAME_REVERSE_MAPPING[nt_type]
                deserialize_fn = NT_TYPE_NAME_DESERIALIZE_MAPPING[nt_type_code]
                values = deserialize_fn(raw_values)
                if nt_type_code not in NT_ARRAY_TYPES:
                    if length != 1:
                        warnings.warn(
                            f"Found invalid NT length '{length}' in {path} on row {index}. Should be 1."
                        )
                        continue
                    assert type(values) == list
                    values = values[0]
                table[nt_path] = (nt_type, values)
        return table
