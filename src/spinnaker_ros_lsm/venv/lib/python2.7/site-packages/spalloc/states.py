"""Defines the states a job may be in according to the protocol."""

from enum import IntEnum


class JobState(IntEnum):
    """All the possible states that a job may be in."""

    unknown = 0
    """The job ID requested was not recognised"""

    queued = 1
    """The job is waiting in a queue for a suitable machine"""

    power = 2
    """The boards allocated to the job are currently being powered on or
    powered off.
    """

    ready = 3
    """The job has been allocated boards and the boards are not currently
    powering on or powering off.
    """

    destroyed = 4
    """The job has been destroyed"""
