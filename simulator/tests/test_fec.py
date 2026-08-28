from roboprotocol_sim.protocol.fec import FlexFecTracker


def test_block_recovers_when_losses_at_or_below_redundancy():
    tracker = FlexFecTracker()
    block_size, n_redundant = 10, 3
    result = None
    # 3 losses out of 13 total datagrams == exactly at the redundancy budget.
    for i in range(block_size + n_redundant):
        lost = i < 3
        result = tracker.record(block_id=1, block_size=block_size, n_redundant=n_redundant, lost=lost, t=float(i))
    assert result is not None
    assert result.recovered is True
    assert result.n_lost == 3


def test_block_fails_when_losses_exceed_redundancy():
    tracker = FlexFecTracker()
    block_size, n_redundant = 10, 3
    result = None
    for i in range(block_size + n_redundant):
        lost = i < 4  # one more loss than the redundancy budget covers
        result = tracker.record(block_id=2, block_size=block_size, n_redundant=n_redundant, lost=lost, t=float(i))
    assert result is not None
    assert result.recovered is False
    assert result.n_lost == 4


def test_blocks_are_tracked_independently_and_out_of_order():
    tracker = FlexFecTracker()
    # Interleave two blocks' reports; block 1 should still finalize once its own
    # count is complete, independent of block 2's progress.
    r = None
    for i in range(9):
        r = tracker.record(block_id=1, block_size=5, n_redundant=2, lost=False, t=0.0)
        tracker.record(block_id=2, block_size=5, n_redundant=2, lost=False, t=0.0)
        if r is not None:
            break
    assert r is not None
    assert r.block_id == 1
    assert r.recovered is True
