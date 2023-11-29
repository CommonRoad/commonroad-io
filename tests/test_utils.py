import os


def get_test_directory():
    cur_dir = os.getcwd()
    scr_dir = cur_dir + "/../tests"
    if os.path.exists(scr_dir):
        return scr_dir
    scr_dir = cur_dir + "/../../tests"
    if os.path.exists(scr_dir):
        return scr_dir
    scr_dir = cur_dir + "/tests"
    if os.path.exists(scr_dir):
        return scr_dir
    else:
        raise RuntimeError(f"get_test_directory: No dir found! cwd: {cur_dir}")
