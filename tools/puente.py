# Puente PC <-> Cerebro P4 (por el cable USB, red 192.168.7.x).
#   1) Toma la cámara del notebook con ffmpeg, la comprime en JPEG y se la manda al P4.
#   2) Le da internet al P4: deja conexiones abiertas hacia él; cuando el P4 pide
#      "CONNECT host:puerto", este programa abre esa conexión y pasa los bytes (el P4 cifra
#      el HTTPS él mismo con su hardware).
#   3) Cada 2 s muestra en pantalla lo que mide el P4 (fps, CPU, aceleradores, temperatura).
# Todas las conexiones las abre el PC: Windows no pide permisos de firewall ni de administrador.
#
# Uso:  python puente.py                 (cámara HP a 1920x1080, 30 fps)
#       python puente.py --sin-camara    (sólo internet y medición)
#       python puente.py --camara "HP 5MP Camera" --calidad 5
import argparse, json, socket, struct, subprocess, sys, threading, time, urllib.request, webbrowser, shutil

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")   # la consola de Windows no siempre es UTF-8
except Exception:
    pass

P4 = "192.168.7.1"
PORT_FRAMES, PORT_TUNNEL = 5000, 5001
MAGIC = 0x464A3450  # "P4JF"

ap = argparse.ArgumentParser()
ap.add_argument("--camara", default="HP 5MP Camera")
ap.add_argument("--ancho", type=int, default=1920)
ap.add_argument("--alto", type=int, default=1080)
ap.add_argument("--fps", type=int, default=30)
ap.add_argument("--calidad", type=int, default=5, help="calidad JPEG de ffmpeg: 2 = mejor, 31 = peor")
ap.add_argument("--sin-camara", action="store_true")
ap.add_argument("--abrir", action="store_true", help="abrir el dashboard en el navegador")
args = ap.parse_args()

stats = {"enviados": 0, "bytes": 0, "descartados": 0, "tuneles": 0, "tunel_bytes": 0}
lock = threading.Lock()


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def esperar_p4():
    t0 = time.time()
    while True:
        try:
            with socket.create_connection((P4, 80), timeout=2):
                return
        except OSError:
            if time.time() - t0 > 3:
                log("esperando al P4 en", P4, "(¿cable en el puerto HUSB?)")
                t0 = time.time()
            time.sleep(1)


# ---------------- túnel a internet ----------------
def tubo(a, b, cuenta):
    try:
        while True:
            d = a.recv(65536)
            if not d:
                break
            b.sendall(d)
            if cuenta:
                with lock:
                    stats["tunel_bytes"] += len(d)
    except OSError:
        pass
    finally:
        for s in (a, b):
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


def tunel_worker():
    while True:
        try:
            p4 = socket.create_connection((P4, PORT_TUNNEL), timeout=5)
            p4.settimeout(None)
            linea = b""
            while not linea.endswith(b"\n"):
                c = p4.recv(1)
                if not c:
                    raise OSError("el P4 cerró")
                linea += c
                if len(linea) > 300:
                    raise OSError("pedido raro")
            txt = linea.decode("ascii", "replace").strip()
            if not txt.startswith("CONNECT "):
                p4.close()
                continue
            host, _, port = txt[8:].rpartition(":")
            port = int(port)
            if port not in (80, 443):
                p4.sendall(b"ERR puerto no permitido\n")
                p4.close()
                continue
            try:
                up = socket.create_connection((host, port), timeout=10)
                up.settimeout(None)
            except OSError as e:
                p4.sendall(("ERR %s\n" % e).encode())
                p4.close()
                continue
            p4.sendall(b"OK\n")
            with lock:
                stats["tuneles"] += 1
            t = threading.Thread(target=tubo, args=(up, p4, True), daemon=True)
            t.start()
            tubo(p4, up, False)
            t.join(timeout=30)
            up.close()
            p4.close()
        except OSError:
            time.sleep(1)


# ---------------- cámara -> P4 ----------------
ultimo = {"jpg": None, "seq": 0}
hay_nuevo = threading.Event()


def lector_ffmpeg():
    ff = shutil.which("ffmpeg")
    if not ff:
        log("no encuentro ffmpeg")
        return
    cmd = [ff, "-hide_banner", "-loglevel", "error", "-f", "dshow", "-rtbufsize", "256M",
           "-video_size", f"{args.ancho}x{args.alto}", "-framerate", str(args.fps), "-pixel_format", "nv12",
           "-i", f"video={args.camara}", "-c:v", "mjpeg", "-q:v", str(args.calidad), "-pix_fmt", "yuvj420p",
           "-f", "image2pipe", "-"]
    while True:
        log("abriendo la cámara:", args.camara, f"{args.ancho}x{args.alto}@{args.fps}")
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=0)
        buf = b""
        while True:
            d = p.stdout.read(262144)
            if not d:
                break
            buf += d
            while True:
                fin = buf.find(b"\xff\xd9")
                if fin < 0:
                    break
                ini = buf.find(b"\xff\xd8")
                jpg = buf[ini:fin + 2] if 0 <= ini < fin else None
                buf = buf[fin + 2:]
                if jpg:
                    with lock:
                        if ultimo["jpg"] is not None:
                            stats["descartados"] += 1
                        ultimo["jpg"] = jpg
                        ultimo["seq"] += 1
                    hay_nuevo.set()
        log("ffmpeg terminó (código", p.wait(), "); reintento en 3 s")
        time.sleep(3)


def emisor():
    while True:
        try:
            s = socket.create_connection((P4, PORT_FRAMES), timeout=5)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
            s.settimeout(10)
            log("mandando video al P4")
            while True:
                hay_nuevo.wait()
                with lock:
                    jpg, seq = ultimo["jpg"], ultimo["seq"]
                    ultimo["jpg"] = None
                    hay_nuevo.clear()
                if jpg is None:
                    continue
                s.sendall(struct.pack("<4I", MAGIC, len(jpg), seq & 0xFFFFFFFF, 0) + jpg)
                with lock:
                    stats["enviados"] += 1
                    stats["bytes"] += len(jpg)
        except OSError as e:
            log("conexión de video cortada:", e)
            time.sleep(1)


# ---------------- medición ----------------
def monitor():
    prev = dict(stats)
    t_prev = time.time()
    while True:
        time.sleep(2)
        now = time.time()
        dt = now - t_prev
        with lock:
            cur = dict(stats)
        fps = (cur["enviados"] - prev["enviados"]) / dt
        mbps = (cur["bytes"] - prev["bytes"]) * 8 / 1e6 / dt
        prev, t_prev = cur, now
        try:
            s = json.loads(urllib.request.urlopen(f"http://{P4}/api/stats", timeout=2).read())
            e = s["eng"]
            linea = (f"PC→P4 {fps:4.1f} fps {mbps:5.1f} Mb/s | P4 sale {s['fps_out']:4.1f} fps | "
                     f"CPU {s['cpu'][0]:3.0f}/{s['cpu'][1]:3.0f}% | JPEGdec {e['jpeg_dec']:3.0f}% JPEGenc {e['jpeg_enc']:3.0f}% "
                     f"PPA {e['ppa']:3.0f}% H264 {e['h264']:3.0f}% USB {e['usb']:3.0f}% DMA2D {e.get('dma2d', 0):3.0f}% | "
                     f"IA caras {s['ai']['face_fps']:4.1f}/s personas {s['ai']['person_fps']:4.1f}/s gatos {s['ai']['cat_fps']:4.1f}/s | "
                     f"{s['temp']:4.1f}°C | caras {s['ai']['faces']} personas {s['ai']['people']} gatos {s['ai']['cats']}")
        except Exception as ex:
            linea = f"PC→P4 {fps:4.1f} fps {mbps:5.1f} Mb/s | (sin respuesta del P4: {ex})"
        log(linea)


if __name__ == "__main__":
    log("Cerebro P4 — puente del PC")
    esperar_p4()
    log("P4 encontrado en", P4)
    for _ in range(4):
        threading.Thread(target=tunel_worker, daemon=True).start()
    if not args.sin_camara:
        threading.Thread(target=lector_ffmpeg, daemon=True).start()
        threading.Thread(target=emisor, daemon=True).start()
    if args.abrir:
        webbrowser.open(f"http://{P4}")
    try:
        monitor()
    except KeyboardInterrupt:
        log("chao")
