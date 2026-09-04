import base64
import json
import time


def message(kind, payload, request_id=None):
    return {"type": kind, "timestamp": time.time(), "request_id": request_id, "payload": payload}


def encode(value):
    return json.dumps(value, separators=(",", ":"))


def test_image_data():
    # Tiny valid PNG, sufficient to validate an HMI image pipeline.
    return base64.b64encode(bytes.fromhex(
        "89504e470d0a1a0a0000000d49484452000000010000000108060000001f15c489"
        "0000000d49444154789c6360f8cf00000003000101c9fe92ef0000000049454e44ae426082"
    )).decode("ascii")
