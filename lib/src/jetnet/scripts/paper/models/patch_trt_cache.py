import struct
import sys

# Remark: included .cache files are already patched!!!

input_file = sys.argv[1]
output_file = sys.argv[2]

with open(input_file) as f:
    lines = f.read().splitlines()

# modify input tensor scale factor by multiplying it with a factor 2
for i, line in enumerate(lines):
    if line.startswith('data:'):
        hex_str_value = line.split(':')[1].strip()
        float_value = struct.unpack('!f', bytes.fromhex(hex_str_value))[0]
        new_float_value = float_value * 2
        print(float_value, new_float_value)
        new_hex_str_value = struct.pack('!f', new_float_value).hex()
        lines[i] = 'data: ' + new_hex_str_value
        break

with open(output_file, "w") as f:
    f.write("\n".join(lines))
