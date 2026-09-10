import json
from enum import Enum

from vector import Vector

def __sanitize_key(obj):
    # keys must be str, int, float, bool or None
    if obj is None: return obj
    if isinstance(obj, str): return obj
    if isinstance(obj, int): return obj
    if isinstance(obj, bool): return obj
    if isinstance(obj, float): return obj
    return str(obj)

def sanitize_keys(obj):
    return {__sanitize_key(k): v for k, v in obj.items()}

def to_json_dict(obj):
    if obj is None: return obj
    if isinstance(obj, str): return obj
    if isinstance(obj, int): return obj
    if isinstance(obj, bool): return obj
    if isinstance(obj, float): return obj
    if isinstance(obj, Enum): return obj.value
    if isinstance(obj, tuple):
        return to_json_dict(list(obj))
    if isinstance(obj, set):
        return to_json_dict(list(obj))

    if isinstance(obj, list):
        result = []
        for e in obj:
            result.append(to_json_dict(e))
        return result
    
    if isinstance(obj, dict):
        result = {}
        for key in obj:
            if isinstance(key, str) or isinstance(key, int) or isinstance(key, bool):
                result[key] = to_json_dict(obj[key])
            else:
                result[str(key)] = to_json_dict(obj[key])
        return result
    
    if hasattr(obj, "__dict__"):    
        d = {}
        for k, v in obj.__dict__.items():
            if not k.startswith("_"):
                d[k] = v
        return to_json_dict(d)

    return str(obj)



class JSON(json.JSONEncoder):
    @classmethod
    def stringify(self, obj):
        return json.dumps(obj, cls=JSON)
    
    def default(self, obj):
        if isinstance(obj, tuple):
            return str(tuple)
        if isinstance(obj, Vector):
            return {"x": obj.x, "y": obj.y}
        return super().default(obj)
