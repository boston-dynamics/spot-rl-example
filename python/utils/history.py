# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

import numpy


class History:
    """Convenience class to record past values of an array of data and calculate statistics on it"""

    def __init__(self) -> None:
        self._data = []

    def record(self, datum):
        """adds a new data entry to the history. i.e. adds a row to the history table

        arguments
        datum -- list of new values to store
        """
        self._data.append(datum)

    def data(self, index):
        """return the list values for a single piece of data in each entry.
            i.e. return one column of the history table

        arguments
        index -- position of desired data in each data entry
        """
        return [row[index] for row in self._data]

    @property
    def mean(self):
        """calculate the mean of each piece of data over the entire history

        return List containing the mean value of each column
        """
        return numpy.mean(self._data, axis=0)

    @property
    def standard_deviation(self):
        """calculate the standard deviation of each piece of data over the entire history

        return List containing the standard deviations of each column
        """
        return numpy.std(self._data, axis=0)
