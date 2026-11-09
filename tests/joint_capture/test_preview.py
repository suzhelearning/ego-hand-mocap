import threading
import time
import pytest
from joint_capture.preview import EgoPreview


def test_preview_rejects_stale_frame_and_bad_camera():
    p=EgoPreview.__new__(EgoPreview);p.lock=threading.Lock();p.errors={};p.frames={0:(time.monotonic(),b'jpeg')}
    assert p.frame(0)==b'jpeg'
    p.frames[0]=(time.monotonic()-2,b'old')
    with pytest.raises(RuntimeError):p.frame(0)
    with pytest.raises(RuntimeError):p.frame(1)
    with pytest.raises(ValueError):p.frame(4)
