import yaml
from tf2_ros import Buffer, TransformListener


class TfFramesBridge:
    """Discovers TF frame names from both dynamic and latched static transforms."""

    def __init__(self, node):
        self._buffer = Buffer(node=node)
        self._listener = TransformListener(self._buffer, node)

    def snapshot(self):
        graph = yaml.safe_load(self._buffer.all_frames_as_yaml())
        if not graph:
            return {'frames': []}
        frames = set(graph)
        # TF's graph lists children as keys; roots appear only as parents.
        frames.update(details['parent'] for details in graph.values())
        return {'frames': sorted(frames)}
