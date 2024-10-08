# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.

import argparse
import contextlib
import itertools

import pytest

from bdai_ros2_wrappers.utilities import Tape, either_or, ensure, namespace_with


def test_tape_content_iteration() -> None:
    tape: Tape[int] = Tape()
    expected_sequence = list(range(10))
    for i in expected_sequence:
        tape.write(i)
    assert list(tape.content()) == expected_sequence


def test_tape_content_destructive_iteration() -> None:
    tape: Tape[int] = Tape()
    expected_sequence = list(range(10))
    for i in expected_sequence:
        tape.write(i)
    assert list(tape.content(expunge=True)) == expected_sequence
    assert len(list(tape.content())) == 0


def test_tape_content_greedy_iteration() -> None:
    tape: Tape[int] = Tape()
    expected_sequence = list(range(10))
    for i in expected_sequence:
        tape.write(i)
    assert tape.content(greedy=True) == expected_sequence


def test_tape_content_following() -> None:
    tape: Tape[int] = Tape()
    expected_sequence = list(range(10))
    for i in expected_sequence:
        tape.write(i)
    with contextlib.closing(tape.content(follow=True)) as stream:
        assert list(itertools.islice(stream, 10)) == expected_sequence
        tape.write(10)
        assert next(stream) == 10


def test_tape_content_greedy_following() -> None:
    tape: Tape[int] = Tape()
    expected_sequence = list(range(10))
    for i in expected_sequence:
        tape.write(i)
    with contextlib.closing(tape.content(greedy=True, follow=True)) as stream:
        assert next(stream) == expected_sequence
        tape.write(10)
        tape.write(20)
        assert next(stream) == [10, 20]


def test_tape_drops_unused_streams() -> None:
    tape: Tape[int] = Tape(max_length=0)

    stream = tape.content(follow=True)
    expected_value = 42
    tape.write(expected_value)
    value = next(stream)
    assert value == expected_value

    del stream

    assert len(tape._streams) == 0


def test_either_or() -> None:
    assert either_or(None, "value", True)
    data = argparse.Namespace(value=True)
    assert either_or(data, "value", False)
    data = argparse.Namespace(value=True, getter=lambda obj: obj.value)
    assert either_or(data, "getter", False)


def test_namespace_with() -> None:
    assert namespace_with(None, "foo") == "foo"
    assert namespace_with("", "foo") == "foo"
    assert namespace_with("/", "foo") == "/foo"
    assert namespace_with("foo", "bar") == "foo/bar"


def test_ensure() -> None:
    data = None
    with pytest.raises(ValueError) as excinfo:
        ensure(data)
    assert "ensure(data) failed" in str(excinfo.value)
