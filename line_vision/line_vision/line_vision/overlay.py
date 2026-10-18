import cv2
import numpy as np


def draw(obs, roi_rect, lines):
    proc = obs.debug["proc"].copy()
    x0, y0, x1, y1 = roi_rect
    roi = proc[y0:y1, x0:x1]
    tint = [(obs.debug.get("line"), (255, 0, 255)), (obs.debug.get("main"), (255, 0, 0)),
            (obs.debug.get("silver"), (200, 200, 0)), (obs.debug.get("red"), (0, 0, 255)),
            (obs.debug.get("green"), (0, 255, 0))]
    for mask, colour in tint:
        if mask is not None:
            roi[mask > 0] = (0.5 * roi[mask > 0] + 0.5 * np.array(colour)).astype(np.uint8)
    cv2.rectangle(proc, (x0, y0), (x1 - 1, y1 - 1), (0, 255, 255), 1)

    def p(x, y):
        return int(x + x0), int(y + y0)

    for x, y, _ in obs.track:
        cv2.circle(proc, p(x, y), 3, (0, 255, 255), -1)
    robot = p(obs.roi_w / 2, obs.roi_h - 1)
    if obs.entry:
        cv2.circle(proc, p(obs.entry.x, obs.entry.y), 6, (255, 255, 255), 2)
    for e in obs.exits:
        cv2.arrowedLine(proc, robot, p(e.x, e.y), (0, 140, 255), 2, tipLength=0.08)
        cv2.putText(proc, e.side[0].upper(), p(e.x, e.y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 140, 255), 1)
    for m in obs.markers:
        colour = (0, 255, 0) if m.before_crossing else (128, 128, 128)
        tl = p(m.x - m.w / 2, m.y - m.h / 2)
        cv2.rectangle(proc, tl, p(m.x + m.w / 2, m.y + m.h / 2), colour, 2)
        cv2.putText(proc, (m.side or "?")[0].upper(), tl, cv2.FONT_HERSHEY_SIMPLEX, 0.4, colour, 1)
    for strip, colour in ((obs.red_strip, (0, 0, 255)), (obs.silver_strip, (255, 255, 0))):
        if strip:
            cv2.rectangle(proc, p(strip.x, strip.y), p(strip.x + strip.w, strip.y + strip.h), colour, 2)

    scale = 2
    out = cv2.resize(proc, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    for i, text in enumerate(lines):
        org = (6, 18 + 18 * i)
        cv2.putText(out, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(out, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return out
