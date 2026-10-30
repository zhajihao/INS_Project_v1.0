# -*- coding: utf-8 -*-
# B2: R2 pointer star reposition: "type* name" -> "type *name", bytes-level regex
import re, os

BASE = r"d:\zjh_project\Combined_Navigation_Project\software\2026-10-23_v1.0.7_baseline\IMU_Project"
DIRS = [BASE + r"\User", BASE + r"\Navi", BASE + r"\Peripheral"]

TYPES = (rb"struct\s+\w+|float|double|char|int|void|"
         rb"uint8_t|uint16_t|uint32_t|int8_t|int16_t|int32_t")
PAT = re.compile(rb"\b(" + TYPES + rb")\*\s*(?=\w)")

grand = 0
for d in DIRS:
    for root, _, files in os.walk(d):
        for fn in sorted(files):
            if not fn.endswith((".c", ".h")):
                continue
            p = os.path.join(root, fn)
            raw = open(p, "rb").read()
            hits = []
            def repl(m):
                hits.append(m.group(0))
                return m.group(1) + b" *"
            new = PAT.sub(repl, raw)
            if hits:
                open(p, "wb").write(new)
                grand += len(hits)
                rel = os.path.relpath(p, BASE)
                print("%-55s %3d处" % (rel, len(hits)))
print("TOTAL %d处" % grand)
