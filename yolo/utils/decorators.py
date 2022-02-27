import time

def exception(func):
    def wrapper(*args):
        try:
            func(*args)
        except Exception as e:
            print(e)
    return wrapper

def timer(msg="Time elapsed"):
    def decorator(func):
        def wrapper(*args):
            start = time.time()
            result = func(*args)
            print("{} : {}".format(msg, time.time()-start))
            return result
        return wrapper
    return decorator

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
