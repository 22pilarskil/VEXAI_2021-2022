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
            func(*args)
            print("{} : {}".format(msg, time.time()-start))
        return wrapper
    return decorator