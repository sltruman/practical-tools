import os

def get_object_path(name):
    return os.path.join(os.path.dirname(__file__),name)
