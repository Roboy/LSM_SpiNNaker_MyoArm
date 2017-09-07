from data_specification import exceptions


class MemoryRegionCollection(object):
    """Collection of memory regions.
    """
    def __init__(self, n_regions):
        """Create a new MemoryRegionCollection with the given number of regions.
        """
        self._n_regions = n_regions
        self._regions = [None] * n_regions

    def __len__(self):
        return self._n_regions

    def __getitem__(self, key):
        return self._regions[key]

    def __setitem__(self, key, value):
        if self._regions[key] is not None:
            raise exceptions.DataSpecificationRegionInUseException(key)
        self._regions[key] = value

    def __iter__(self):
        return iter(self._regions)

    @property
    def regions(self):
        for region in self._regions:
            yield region

    def is_empty(self, region):
        return self._regions[region] is None

    def is_unfilled(self, region):
        return self._regions[region].unfilled

    def count_used_regions(self):
        count = 0
        for i in self._regions:
            if i is not None:
                count += 1
        return count

    def needs_to_write_region(self, region_id):
        """
        helper method which determines if a unfilled region actually needs to
        be written (optimisation to stop large data files)

        :param region_id: the region id to which the test is being ran on
        :return: a boolean stating if the region needs to be written
        :rtype: boolean
        :raise DataSpecificationNoRegionSelectedException: when the id is \
        beyond the expected region range
        """
        if region_id > self._n_regions:
            raise exceptions.DataSpecificationNoRegionSelectedException(
                "the region id requested is beyond the supported number of"
                "avilable region ids")
        if not self._regions[region_id].unfilled:
            return True
        else:
            needs_writing = False
            for region in range(region_id, self._n_regions):
                if (self._regions[region_id] is not None
                        and not self._regions[region_id].unfilled):
                    needs_writing = True
        return needs_writing

