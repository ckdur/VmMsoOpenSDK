#!/usr/bin/env python3
"""Extract DDS buffer uploads (EP 0x06) from usbtrace logs and summarise them."""
import sys


def uploads(path):
    out = []
    for line in open(path, errors="replace"):
        if " OUT ep=06 " in line and " : " in line:
            hx = line.split(" : ", 1)[1].split()
            b = bytes(int(x, 16) for x in hx)
            out.append([b[i] | (b[i + 1] << 8) for i in range(0, len(b) - 1, 2)])
    return out


def summary(v):
    n = len(v)
    mn, mx = min(v), max(v)
    return "n=%d min=%d@%d max=%d@%d v0=%d v[n/4]=%d v[n/2]=%d v[3n/4]=%d" % (
        n, mn, v.index(mn), mx, v.index(mx), v[0], v[n // 4], v[n // 2], v[3 * n // 4])


a = uploads(sys.argv[1])
b = uploads(sys.argv[2]) if len(sys.argv) > 2 else []
for i in range(max(len(a), len(b))):
    sa = summary(a[i]) if i < len(a) else "-"
    sb = summary(b[i]) if i < len(b) else "-"
    same = i < len(a) and i < len(b) and a[i] == b[i]
    if same:
        print("#%d identical %s" % (i, sa))
    else:
        nd = sum(1 for x, y in zip(a[i], b[i]) if x != y) if i < len(a) and i < len(b) else -1
        maxd = max((abs(x - y) for x, y in zip(a[i], b[i])), default=0) if nd >= 0 else -1
        print("#%d DIFF (%d samples, max %d)\n   A %s\n   B %s" % (i, nd, maxd, sa, sb))
