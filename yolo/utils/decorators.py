import time

class bcolors:

    color_dict = {
        'violet':'\033[95m',
        'blue':'\033[94m',
        'cyan':'\033[96m',
        'green':'\033[92m',
        'yellow':'\033[93m',
        'red':'\033[91m',
        'bold':'\033[1m',
        'underline':'\033[4m',
        'end':'\033[0m'
    }

    @staticmethod
    def print(string, color):
        print(bcolors.color_dict[color] + string + bcolors.color_dict['end'])

def exception(func):
    def wrapper(*args):
        try:
            func(*args)
        except Exception as e:
            bcolors.print(str(e), "red")
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