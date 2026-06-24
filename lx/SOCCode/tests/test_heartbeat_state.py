#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Heartbeat STATE parsing and takeover gate tests."""

import threading
import time

from heartbeat import (
    HB_STATE_ACTIVE_CONTROL,
    HB_STATE_BOOTING,
    HB_STATE_NO_INPUT_IDLE,
    PeerHeartbeat,
    _parse_primary_hb_fields,
)


def test_parse_primary_hb_state_field():
    seed = _parse_primary_hb_fields(
        'HB:1 SEQ:7 STATE:NO_INPUT_IDLE PSI:0.1000 DELTA:0.0200 '
        'ACC:+0.50 AEB:0 CLS:3'
    )
    assert seed is not None
    psi, delta, acc, aeb, cls, seq, state = seed
    assert psi == 0.1
    assert delta == 0.02
    assert acc == 0.5
    assert aeb == 0
    assert cls == 3
    assert seq == 7
    assert state == HB_STATE_NO_INPUT_IDLE


def test_parse_primary_hb_missing_state_defaults_active_control():
    seed = _parse_primary_hb_fields(
        'HB:1 SEQ:8 PSI:0.1000 DELTA:0.0200 ACC:+0.50 AEB:0 CLS:1'
    )
    assert seed is not None
    assert seed[-1] == HB_STATE_ACTIVE_CONTROL


def _heartbeat_without_threads(state, armed):
    hb = PeerHeartbeat.__new__(PeerHeartbeat)
    hb._lock = threading.Lock()
    hb._takeover = False
    hb._advertise_active = False
    hb._takeover_armed = armed
    hb._last_primary_state = state
    hb.peer_last_rx = time.monotonic() - 10.0
    hb._last_seq_change_t = time.monotonic() - 10.0
    hb._last_rx_seq = 1
    return hb


def test_no_input_idle_does_not_takeover_even_when_silent():
    hb = _heartbeat_without_threads(HB_STATE_NO_INPUT_IDLE, armed=False)
    hb._check_takeover()
    assert hb._takeover is False


def test_active_control_silence_triggers_takeover_when_armed():
    hb = _heartbeat_without_threads(HB_STATE_ACTIVE_CONTROL, armed=True)
    hb._check_takeover()
    assert hb._takeover is True


def test_booting_does_not_takeover_even_if_armed_flag_was_set():
    hb = _heartbeat_without_threads(HB_STATE_BOOTING, armed=True)
    hb._check_takeover()
    assert hb._takeover is False

