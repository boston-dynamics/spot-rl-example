# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

from typing import Any, List


def dict_from_lists(keys: List, values: List):
    """construct a dict from two lists where keys and associated values appear at same index

    arguments
    keys -- list of keys
    values -- list of values in same order as keys

    return dictionary mapping keys to values
    """

    return dict(zip(keys, values))


def dict_to_list(data: dict, keys: List):
    """construct a list of values from dictionary in the order specified by keys

    arguments
    dict -- dictionary to look up keys in
    keys -- list of keys to retrieve in orer

    return list of values from dict in same order as keys
    """
    return [data.get(key) for key in keys]


def set_matching(data: dict, regex, value):
    """set values in dict with keys matching regex

    arguments
    dict -- dictionary to set keys in
    regex -- regex to select keys that will be set
    value -- value to set keys matching regex to
    """
    for key in data:
        if regex.match(key):
            data[key] = value


def reorder(inputs: List[Any], ordering: List[int]) -> List[Any]:
    """rearrange values in a list to a given order.

    arguments
    inputs -- list of values
    ordering -- list of len(inputs) containing ints from 0 - len(inputs)-1 in desired order

    return list of values in new order
    """
    return [inputs[i] for i in ordering]


def find_ordering(input: List[Any], output: List[Any]) -> List[int]:
    """given two lists containing the same values return a list of indices mapping input->output

    arguments
    input -- first list of values
    output -- list with same values as input in a different ordering

    return list such that the nth value is the index of output[n] in the input list
    """
    return [input.index(key) for key in output]
