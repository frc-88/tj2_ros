source = b"abcdefghijklmnopqrstuvwxyz0123456789\n"
result = b"O\xa7NS\xd3\xa6\xa6\xa9i)\xe9R\xd2\xa4\xa4\xd4\xb4\x94tT4\x14\xf4\xa8h\xe8\xd6\xb6\x96vV6\x16\x00"

def to_binary_str(string: bytes):
    return ("{:08b}" * len(string)).format(*tuple([x for x in string]))

source_bin = to_binary_str(source)
result_bin = to_binary_str(result)

print(result_bin)
for char in source:
    binary = "{:08b}".format(char)
    print(binary)
    index = result_bin.find(binary)
    if index < 0:
        print("%s not in result" % char.to_bytes(1, "big"))
    else:
        result_bin = result_bin[0:index] + result_bin[index + 8:]
print(result_bin)
