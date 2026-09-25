# Lee el puerto serie del P4 (FUSB) por N segundos y opcionalmente manda comandos.
# Uso: python serial_log.py COM6 30 [t:cmd ...]   ej:  python serial_log.py COM6 40 20:m 35:s
import sys, time, serial

port, dur = sys.argv[1], float(sys.argv[2])
cmds = []
for a in sys.argv[3:]:
    t, c = a.split(":", 1)
    cmds.append((float(t), c))
s = serial.Serial(port, 115200, timeout=0.2)
t0 = time.time()
buf = b""
while time.time() - t0 < dur:
    now = time.time() - t0
    while cmds and cmds[0][0] <= now:
        s.write(cmds.pop(0)[1].encode())
    d = s.read(4096)
    if d:
        buf += d
        *lines, buf = buf.split(b"\n")
        for ln in lines:
            txt = ln.decode("utf-8", "replace").rstrip("\r")
            txt = txt.replace("\x1b[0;32m", "").replace("\x1b[0;33m", "").replace("\x1b[0;31m", "").replace("\x1b[0m", "")
            print(txt, flush=True)
s.close()
