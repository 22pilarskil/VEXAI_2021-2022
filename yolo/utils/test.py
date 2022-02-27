def num():
	globals()[numpy] = __import__(numpy)

num()

print(np.random.randn(1))