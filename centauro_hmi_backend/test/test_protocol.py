import json

from centauro_hmi_backend.protocol import encode, message, test_image_data as make_test_image_data


def test_message_contains_envelope_and_request_id():
    result = message('ack', {'accepted': True}, 'req-1')

    assert result['type'] == 'ack'
    assert result['request_id'] == 'req-1'
    assert result['payload'] == {'accepted': True}
    assert isinstance(result['timestamp'], float)


def test_encode_returns_compact_json_round_trip():
    raw = encode(message('telemetry', {'value': 1}))

    assert json.loads(raw)['payload']['value'] == 1
    assert ' ' not in raw


def test_image_data_is_a_valid_png_signature():
    import base64

    image = base64.b64decode(make_test_image_data())
    assert image.startswith(b'\x89PNG\r\n\x1a\n')
