import sys
import numpy as np

DEFAULT_PATH = '/home/psb120/Documents/dobot-tcp-ip/iros/20260228-221619.npz'

def main(path=DEFAULT_PATH):
	d = np.load(path, allow_pickle=True)
	keys = getattr(d, 'files', None)
	if keys:
		print(d[keys[0]])
	else:
		print(dict(d))

if __name__ == '__main__':
	main(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PATH)
