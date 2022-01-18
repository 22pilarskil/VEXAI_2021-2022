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
