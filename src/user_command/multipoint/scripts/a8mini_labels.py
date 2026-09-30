"""Class names and drawing for the bundled A8 mini YOLO detector."""

import json
from pathlib import Path
import re


def load_class_names(model_path):
    """Load and validate the names shipped alongside an engine."""
    path = Path(model_path).with_suffix(".names.json")
    with path.open(encoding="utf-8") as stream:
        data = json.load(stream)
    names = data.get("names", data) if isinstance(data, dict) else data
    if isinstance(names, (list, tuple)):
        names = dict(enumerate(names))
    if not isinstance(names, dict) or not names:
        raise ValueError("invalid class-name table: " + str(path))
    normalized = {}
    for key, value in names.items():
        if not re.fullmatch(r"0|[1-9][0-9]*", str(key)):
            raise ValueError("invalid class ID %r in %s" % (key, path))
        index = int(key)
        if (index in normalized or not isinstance(value, str) or not value.strip()
                or re.fullmatch(r"class[_ ]?\d+", value.strip(), re.IGNORECASE)):
            raise ValueError("invalid class name for ID %s in %s" % (key, path))
        normalized[index] = value.strip()
    if set(normalized) != set(range(len(normalized))):
        raise ValueError("class IDs must be contiguous from zero in " + str(path))
    return normalized


def draw_detection(frame, box, cls_id, conf, names, cv2):
    """Draw a clipped box and its class label on an OpenCV frame."""
    if cls_id not in names:
        raise ValueError("detected class ID %d is absent from the class-name table" % cls_id)
    frame_height, frame_width = frame.shape[:2]
    x1, y1, x2, y2 = map(int, box)
    x1 = max(0, min(x1, frame_width - 1))
    y1 = max(0, min(y1, frame_height - 1))
    x2 = max(0, min(x2, frame_width - 1))
    y2 = max(0, min(y2, frame_height - 1))
    label = "%s %.2f" % (names[cls_id], conf)
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    (text_width, text_height), baseline = cv2.getTextSize(
        label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    label_bottom = y1 - 5
    if label_bottom - text_height - baseline < 0:
        label_bottom = y1 + text_height + baseline + 5
    label_left = x1
    label_right = min(label_left + text_width + 6, frame_width - 1)
    label_top = max(0, label_bottom - text_height - baseline - 6)
    cv2.rectangle(frame, (label_left, label_top), (label_right, label_bottom + 3),
                  (0, 255, 0), -1)
    cv2.putText(frame, label, (label_left + 3, label_bottom - baseline),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)
