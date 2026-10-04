#!/usr/bin/env bash
set +e
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$repo" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 - <<'PY'
import importlib
import inspect

def show(name):
    try:
        obj = importlib.import_module(name)
        print(f'{name}.import=OK')
        print(f'{name}.file={getattr(obj, "__file__", "<no __file__>")}')
        return obj
    except Exception as exc:
        print(f'{name}.import=ERROR:{type(exc).__name__}:{exc}')
        return None

transport = show('gz.transport13')
pose = show('gz.msgs10.pose_pb2')
if transport is not None:
    node_cls = getattr(transport, 'Node', None)
    print(f'Node.present={node_cls is not None}')
    if node_cls is not None:
        print(f'Node.file={inspect.getfile(node_cls)}')
        for method_name in ('request', 'subscribe'):
            method = getattr(node_cls, method_name, None)
            print(f'Node.{method_name}.present={method is not None}')
            if method is not None:
                try:
                    print(f'Node.{method_name}.signature={inspect.signature(method)}')
                except Exception as exc:
                    print(f'Node.{method_name}.signature_unavailable={type(exc).__name__}:{exc}')
                doc = inspect.getdoc(method) or getattr(method, '__doc__', None)
                print(f'Node.{method_name}.doc={doc!r}')
                try:
                    print(f'Node.{method_name}.file={inspect.getfile(method)}')
                except Exception as exc:
                    print(f'Node.{method_name}.file_unavailable={type(exc).__name__}:{exc}')
if pose is not None:
    print(f'Pose.present={getattr(pose, "Pose", None) is not None}')
    if getattr(pose, 'Pose', None) is not None:
        print(f'Pose.file={inspect.getfile(pose.Pose)}')
PY
