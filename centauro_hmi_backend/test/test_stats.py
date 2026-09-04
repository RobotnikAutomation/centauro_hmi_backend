from centauro_hmi_backend.stats import InboundStats, compact_payload


def test_stats_count_and_reset_interval():
    stats = InboundStats()
    stats.record('websocket', 'arm.joint_jog', '{}', {'velocities': [1, 0, 0, 0, 0, 0]})
    stats.record('ros2', None, 'bad', error=True)

    first = stats.take_interval()
    assert first['messages'] == 2
    assert first['total_messages'] == 2
    assert first['errors'] == 1
    assert first['by_source'] == {'websocket': 1, 'ros2': 1}
    assert first['by_command']['arm.joint_jog'] == 1

    second = stats.take_interval()
    assert second['messages'] == 0
    assert second['total_messages'] == 2


def test_compact_payload_limits_output():
    assert len(compact_payload({'value': 'x' * 100}, 30)) == 30
