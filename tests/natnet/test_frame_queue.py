import pytest

from natnet_zenoh.frame_queue import LatestFrameQueue


def frame(number):
    return {"frame_number": number}


def test_queue_preserves_fifo_order_below_capacity():
    frames = LatestFrameQueue(capacity=3)

    assert frames.put_latest(frame(1)) == 0
    assert frames.put_latest(frame(2)) == 0

    assert frames.get(timeout=0.01)["frame_number"] == 1
    assert frames.get(timeout=0.01)["frame_number"] == 2
    assert frames.empty()


def test_queue_rejects_non_positive_capacity():
    with pytest.raises(ValueError, match="capacity"):
        LatestFrameQueue(capacity=0)


def test_queue_drops_exactly_the_oldest_frame_when_full():
    frames = LatestFrameQueue(capacity=2)
    frames.put_latest(frame(1))
    frames.put_latest(frame(2))
    newest = frame(3)

    assert frames.put_latest(newest) == 1

    assert frames.dropped_frames == 1
    assert newest["publisher_dropped_frames"] == 1
    assert frames.get(timeout=0.01)["frame_number"] == 2
    assert frames.get(timeout=0.01)["frame_number"] == 3
