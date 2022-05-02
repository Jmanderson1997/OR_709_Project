import sys
import importlib

loaded_modules = set()
def freeze_modules():
    global loaded_modules
    loaded_modules = set(sys.modules)

def reimport(first_call=True):
    global loaded_modules
    to_reload = set(sys.modules) - loaded_modules

    for module in to_reload:
        if not sys.modules[module]:
            continue

        if not hasattr(sys.modules[module], "__file__"):
            continue

        filename = sys.modules[module].__file__
        try:
            if "site-packages" in filename or "anaconda" in filename:
                continue
        except:
            continue

        importlib.reload(sys.modules[module])

    if first_call:
        reimport(False)