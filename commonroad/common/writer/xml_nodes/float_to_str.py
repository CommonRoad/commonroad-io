from commonroad.common.writer.file_writer_interface import precision


def float_to_str(f):
    """
    Convert the given float to a string,
    without resorting to scientific notation
    """
    fstring = str(f)
    if "e" in fstring:
        return format(f, ".{}f".format(precision.decimals))
    f_list = fstring.split(".")
    if len(f_list) > 1:
        return f_list[0] + "." + f_list[1][: precision.decimals]
    else:
        return f_list[0]
