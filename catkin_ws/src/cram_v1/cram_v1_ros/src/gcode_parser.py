import re

# Based on work from: https://github.com/AndyEveritt/GcodeParser

# open gcode file and store contents as variable

# ideal formmat
cmd = {"no": '', "command": '', "params": '', "comments": ''}


class GcodeParser:
    def __init__(self, file: str):
        self._file_read(file)

    def _file_read(self, file):
        with open(file, 'r') as f:
            self.gcode = f.read()

    def sort(self):
        if self.command[0] == 'G' and self.command[1] in (0, 1, 2, 3):
            self.type = Commands.MOVE
        elif self.command[0] == ';':
            self.type = Commands.COMMENT
        elif self.command[0] == 'T':
            self.type = Commands.TOOLCHANGE
        else:
            self.type = Commands.OTHER


def get_lines(gcode, include_comments=False):
    # Not sure need to figure out what this does
    regex = r'(?!; *.+)(G|M|T|g|m|t)(\d+)(([ \t]*(?!G|M|g|m)\w(".*"|([-\d\.]*)))*)[ \t]*(;[ \t]*(.*))?|;[ \t]*(.+)'
    regex_lines = re.findall(regex, gcode)
    lines = {}
    id = 0
    # Split string of text into individual lines and assign to a dictionary
    for count, line in enumerate(regex_lines):
        if line[0]:
            lines[id] = {}
            lines[id]["ID"] = {id}
            lines[id]["command"] = (line[0].upper(), int(line[1]))
            lines[id]["params"] = split_params(line[2])
            lines[id]["comments"] = line[-2]
            # Add command type. Maybe refactor into a hidden method
            if line[0].upper() == 'G' and int(line[1]) in (0, 1, 2, 3):
                lines[id]["command_type"] = "move"
            elif line[0] == 'T':
                lines[id]["command_type"] = "tool_change"
            else:
                lines[id]["command_type"] = "other"

            id += 1

        elif include_comments:
            lines[id] = {}
            lines[id]["ID"] = {id}
            lines[id]["command"] = (';', None)
            lines[id]["params"] = {}
            lines[id]["comments"] = line[-1]
            lines[id]["command_type"] = "comment"
            id += 1

        else:
            continue

    return lines

# def cmd_tag(lines):
#     for line in lines
#     if lines[:]["command"][0] == 'G' and lines[:]["command"][0] in (0, 1, 2, 3):
#         lines[:]["command_type"] = "move"
#     elif lines[:]["command"][0] == ';':
#         lines[:]["command_type"] = "comment"
#     elif lines[:]["command"][0] == 'T':
#         lines[:]["command_type"] = "tool_change"
#     else:
#         lines[:]["command_type"] = "other"
#
#

def element_type(element: str):
    if re.search(r'"', element):
        return str
    if re.search(r'\..*\.', element):
        return str
    if re.search(r'[+-]?\d*\.\d+', element):
        return float
    return int


def split_params(line):
    regex = r'((?!\d)\w+?)(".*"|(\d+\.?)+|-?\d*\.?\d*)'
    elements = re.findall(regex, line)
    params = {}
    for element in elements:
        params[element[0].upper()] = element_type(element[1])(element[1])

    return params


if __name__ == "__main__":
    parser = GcodeParser('3DBenchy.gcode')
    lines = get_lines(parser.gcode,  include_comments=False)
    # lines = cmd_tag(lines)

    print(lines)
    print(lines[0])
    print(lines[30])
    print(lines[100])
    print(lines[200])
    print(lines[1000])
    print(lines[13071])
