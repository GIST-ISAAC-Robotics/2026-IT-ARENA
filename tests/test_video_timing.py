import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from retime_autonomy_video import frame_schedule


def test_uniform_images_keep_simulation_time():
    np.testing.assert_array_equal(frame_schedule([1., 1.05, 1.10]), [0, 1, 2])


def test_dropped_frame_is_held_not_time_compressed():
    np.testing.assert_array_equal(frame_schedule([1., 1.10, 1.20]), [0, 0, 1, 1, 2])


@pytest.mark.parametrize("stamps", ([1.], [1., 1.], [2., 1.], [1., float('nan')]))
def test_bad_camera_clock_rejected(stamps):
    with pytest.raises(ValueError):
        frame_schedule(stamps)
