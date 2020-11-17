import json
import logging
import os

default_params_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   'default_draw_params.json')
with open(default_params_path) as fp:
    default_params = json.load(fp)


def write_default_params(filename: str):
    with open(filename, 'w') as fp:
        json.dump(default_params, fp, indent=4)


class ParamServer:
    def __init__(self, data=None, warn_default=False):
        self.data = data or {}
        self.warn_default = warn_default

    @staticmethod
    def _resolve_key(map, key):
        if len(key) == 0:
            return None, 0
        d = map
        l_key = list(key)
        # Try to find most special version of element
        while len(l_key) > 0:
            k = l_key.pop(0)
            if k in d.keys():
                d = d[k]
            else:
                d = None
                break
        if d is None and len(key) > 0:
            # If not found, remove first level and try again
            return ParamServer._resolve_key(map, key[1:])
        else:
            return d, len(key)

    def by_callstack(self, call_stack, value):
        if isinstance(value, tuple):
            path = call_stack + value
        else:
            path = call_stack + (value,)
        return self.__getitem__(path)

    def __getitem__(self, item):
        if not isinstance(item, tuple):
            item = tuple(item)

        val, depth = ParamServer._resolve_key(self.data, item)
        val_default, depth_default = ParamServer._resolve_key(default_params,
                                                              item)
        if val is None and val_default is None:
            logging.error('Value for key {} not found!'.format(item))
            return None

        if val_default is not None and val is None:
            if self.warn_default:
                logging.warning('Using default for key {}!'.format(item))
            return val_default

        if val is not None and val_default is None:
            return val

        if val is not None and val_default is not None:
            if depth >= depth_default:
                return val
            else:
                return val_default

    def __setitem__(self, key, value):
        if not isinstance(key, tuple):
            key = (key,)
        d = self.data
        for k in key[:-1]:
            if not isinstance(d, dict):
                raise KeyError(
                        'Key "{}" in path "{}" is not subscriptable!'.format(k,
                                                                             key))
            if k in d.keys():
                d = d[k]
            else:
                d[k] = {}
                d = d[k]
        if not isinstance(d, dict):
            raise KeyError(
                    'Key "{}" in path "{}" is not subscriptable!'.format(k,
                                                                         key))
        d[key[-1]] = value

    def __contains__(self, item):
        return item in self.data

    @staticmethod
    def from_json(fname):
        with open(fname, 'r') as fp:
            data = json.load(fp)
        return ParamServer(data)
