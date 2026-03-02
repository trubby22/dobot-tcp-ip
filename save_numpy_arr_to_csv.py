import sys
import numpy as np

DEFAULT_NPZ = '/home/psb120/Documents/dobot-tcp-ip/iros/20260228-230937.npz'
OUT_CSV = '/home/psb120/Documents/dobot-tcp-ip/kinematics_sample.csv'

def main(npz_path=DEFAULT_NPZ, out_csv=OUT_CSV):
    d = np.load(npz_path, allow_pickle=True)
    keys = getattr(d, 'files', None)
    if keys:
        arr = d[keys[0]]
    else:
        dd = dict(d)
        if 'kinematics' in dd:
            arr = dd['kinematics']
        elif len(dd) == 1:
            arr = next(iter(dd.values()))
        else:
            arr = next(iter(dd.values()))

    arr = np.array(arr, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 7:
        raise ValueError(f"Expected array shape (n,7), got {arr.shape}")

    fmt = ['%.0f'] + ['%.1f'] * (arr.shape[1] - 1)
    np.savetxt(out_csv, arr, delimiter=',', fmt=fmt, header='frame,x,y,z,xr,yr,zr', comments='')
    print(f"Saved CSV to {out_csv}")


if __name__ == '__main__':
    npz_arg = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_NPZ
    out_arg = sys.argv[2] if len(sys.argv) > 2 else OUT_CSV
    main(npz_arg, out_arg)
