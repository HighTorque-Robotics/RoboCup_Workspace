from dbehavior import role


def find_behavior(name):
    try:
        cls = getattr(role, name)
        return cls
    except AttributeError:
        return None
