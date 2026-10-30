# -*- coding: utf-8 -*-
# B1: R1 indent conversion, bytes-level, display-preserving under tab=4
import sys

BASE = r"d:\zjh_project\Combined_Navigation_Project\software\2026-10-23_v1.0.7_baseline\IMU_Project"

to_tab_files = [
    BASE + r"\Navi\navi_api.c",
    BASE + r"\Navi\navi_math\mat_inverse.c",
    BASE + r"\Navi\navi_compensation\GetFlag_GNSS_type.c",
    BASE + r"\Navi\navi_config\navi_types.h",
    BASE + r"\Navi\navi_kalman\kalman15d\kalman15d.h",
]
to_space_files = [
    BASE + r"\Peripheral\SYS\sys.c",
]

def spaces_to_tabs(data: bytes):
    out, changed = [], 0
    for line in data.split(b"\n"):
        i = 0
        while i < len(line) and line[i:i+1] == b" ":
            i += 1
        tabs, rem = divmod(i, 4)
        if tabs > 0:
            newline = b"\t" * tabs + b" " * rem + line[i:]
            if newline != line:
                changed += 1
            out.append(newline)
        else:
            out.append(line)
    return b"\n".join(out), changed

def tabs_to_spaces(data: bytes):
    out, changed = [], 0
    for line in data.split(b"\n"):
        i = 0
        while i < len(line) and line[i:i+1] == b"\t":
            i += 1
        if i > 0:
            changed += 1
        out.append(b"    " * i + line[i:])
    return b"\n".join(out), changed

total = 0
for f in to_tab_files:
    raw = open(f, "rb").read()
    new, n = spaces_to_tabs(raw)
    if new != raw:
        open(f, "wb").write(new)
    total += n
    print("TAB<-SPACE %-60s lines_changed=%d" % (f.split("\\")[-1], n))

for f in to_space_files:
    raw = open(f, "rb").read()
    new, n = tabs_to_spaces(raw)
    if new != raw:
        open(f, "wb").write(new)
    total += n
    print("SPACE<-TAB %-60s lines_changed=%d" % (f.split("\\")[-1], n))

print("TOTAL lines_changed=%d" % total)
