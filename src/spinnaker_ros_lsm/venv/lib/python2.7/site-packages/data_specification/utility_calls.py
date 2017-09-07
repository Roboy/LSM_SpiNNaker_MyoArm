"""
utility calls for interpreting bits of the dsg
"""

from data_specification import constants


def get_region_base_address_offset(app_data_base_address, region):
    """
    helper method which finds the address of the of a given region for the dsg
    :param app_data_base_address: base address for the core
    :param region: the region id we're looking for
    :return:
    """
    return (app_data_base_address +
            constants.APP_PTR_TABLE_HEADER_BYTE_SIZE + (region * 4))

