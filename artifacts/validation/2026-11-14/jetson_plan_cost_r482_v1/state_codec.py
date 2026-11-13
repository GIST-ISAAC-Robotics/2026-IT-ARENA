"""계측용 생성 상태 저장. 코드 대신 클래스의 부모/구조 설정/상태를 저장한다."""
import io
import pickle

def rebuild_structure(base, sectors_json, state):
    from arena_autonomy.known_structure import structure_policy
    cls=structure_policy(base,sectors_json)
    result=cls.__new__(cls)
    result.__dict__.update(state)
    return result

class StatePickler(pickle.Pickler):
    def reducer_override(self,obj):
        cls=type(obj)
        if cls.__module__=='arena_autonomy.known_structure' and cls.__name__.startswith('StructureAware'):
            return rebuild_structure,(cls.__bases__[1],cls.structure_sectors_json,obj.__dict__)
        return NotImplemented

def dumps(value):
    stream=io.BytesIO();StatePickler(stream,protocol=5).dump(value);return stream.getvalue()

def loads(value):return pickle.loads(value)
