#!/usr/bin/env python3
"""Print QA_VIEWS for a black-room scan: views of the reference scan 133554 mapped into the scan's frame.

  q_views.py <scan> <canon.json>     canon.json: written by `q_analyze.py cross` (scan world -> reference world)
"""
import json, sys
import numpy as np
VIEWS = [("tuer", [-4.95, -2.88, -0.2], [-6.95, -2.88, -0.5], 75),
         ("leuchtstaebe", [-0.6, -2.88, 0.0], [1.4, -2.88, -0.1], 75),
         ("leiter", [-2.38, -4.6, 0.3], [-2.38, -6.84, 0.3], 80),
         ("notausgang_tuer", [-2.6, -1.0, 0.0], [-2.6, 1.07, -0.4], 75)]
scan = sys.argv[1]
tf = sys.argv[2] if len(sys.argv) > 2 else sys.exit(__doc__)
M = np.eye(4)
if scan != "20261022133554":
    M = np.linalg.inv(np.array(json.load(open(tf))[scan]))  # ref world -> scan world
f = lambda p: ",".join(f"{v:.3f}" for v in (M @ np.array(p + [1.0]))[:3])
print(";".join(f"{n}:{f(e)}:{f(t)}:{fov}" for n, e, t, fov in VIEWS))
