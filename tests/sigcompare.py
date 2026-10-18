#!/usr/bin/env python3
"""Compare two sigtest outputs numerically.

usage: sigcompare.py vendor.txt ours.txt [rel_tol] [abs_tol]
"""
import math
import sys


def load(path):
    d = {}
    for line in open(path):
        if ":" not in line:
            continue
        k, v = line.split(":", 1)
        d[k.strip()] = v.split()
    return d


def num(s):
    try:
        return float(s.split("=")[-1])
    except ValueError:
        return s


def main():
    a, b = load(sys.argv[1]), load(sys.argv[2])
    rel = float(sys.argv[3]) if len(sys.argv) > 3 else 1e-6
    ab = float(sys.argv[4]) if len(sys.argv) > 4 else 1e-9
    bad = 0
    for k in a:
        if k not in b:
            print("MISSING", k)
            bad += 1
            continue
        va, vb = [num(x) for x in a[k]], [num(x) for x in b[k]]
        if len(va) != len(vb):
            print("LEN", k, len(va), len(vb))
            bad += 1
            continue
        worst, wi = 0.0, -1
        for i, (x, y) in enumerate(zip(va, vb)):
            if isinstance(x, str) or isinstance(y, str):
                if x != y:
                    worst, wi = math.inf, i
                continue
            if math.isnan(x) and math.isnan(y):
                continue
            if math.isinf(x) and x == y:
                continue
            err = abs(x - y)
            scale = max(abs(x), abs(y))
            e = 0.0 if err <= ab else err / scale if scale else math.inf
            if e > worst:
                worst, wi = e, i
        if worst > rel:
            print("DIFF %-24s worst rel %.3g at %d: %s vs %s" % (k, worst, wi, a[k][wi], b[k][wi]))
            bad += 1
    for k in b:
        if k not in a:
            print("EXTRA", k)
    print("%d/%d keys differ" % (bad, len(a)))
    return 1 if bad else 0


sys.exit(main())
