#!/usr/bin/env python
import re
import glob
from collections import defaultdict

uavobject_dir="../../librepilot/build/uavobject-synthetics/flight"
header_file="./minOPOSD_additionals_only/UAVTalk.h"

# objids flight
objids = {}

for f in glob.glob(uavobject_dir + "/*.h"):
    text = open(f).read()
    matches = re.findall(r'#define\s+(\w+_OBJID)\s+(\w+)', text)
    for m in matches:
        name, objid = m
        objids[name] = int(objid, 16)

# for k in sorted(objids.keys()):
    # print k, hex(objids[k])



objids_minoposd = defaultdict(set)

text = open(header_file).read()
matches = re.findall(r'#define (\w+_OBJID)(_\d*)?\s+(\w+)', text)
for m in matches:
    name, variant, objid = m
    objids_minoposd[name].add(int(objid, 16))

for k in sorted(objids_minoposd.keys()):
    ids = objids_minoposd[k]
    if not k in objids:
        print "%s missing from uavobject headers" % k
    elif objids[k] not in ids:
        print "%s mismatch: 0x%X (uavobject) vs %s (minoposd)" % \
                (k, objids[k], ", ".join("0x%X" % i for i in ids))
