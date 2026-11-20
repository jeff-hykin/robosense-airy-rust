import sys, math
for name in sys.argv[1:]:
    a = open(f"rust_{name}.txt").read().split("\n"); b = open(f"cpp_{name}.txt").read().split("\n")
    assert len(a) == len(b)
    maxd = 0.0; other = 0; heads = 0
    for x, y in zip(a, b):
        if x == y: 
            heads += x.startswith("S"); continue
        xs, ys = x.split(), y.split()
        if xs[0] in ("S", "I"):
            other += 1; print("hdr", x, "|", y); continue
        for i in range(3):
            fx, fy = float(xs[i]), float(ys[i])
            if math.isnan(fx) and math.isnan(fy): continue
            maxd = max(maxd, abs(fx - fy))
        if xs[3:] != ys[3:]: other += 1; print(x, "|", y) if other < 5 else None
    print(name, "max xyz diff", maxd, "non-xyz mismatches", other, "scans", sum(l.startswith("S") for l in a), "imu", sum(l.startswith("I") for l in a))
