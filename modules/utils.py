import re

def str2float( s : str ):
    """

        Args:
        ----------
            [NAME]          [TYPE]        [DESCRIPTION]
            (1) s           str           The string which will be parsed to float array

        Returns:
        ----------
            A list of float that is parsed from given string s

    """

    return [ float( i ) for i in re.findall( r"[-+]?\d*\.\d+|[-+]?\d+", s ) ]
