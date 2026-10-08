# Usage: python cap.py <port> <seconds> [commands] [outfile]
# Sends each character of <commands> (with a short gap), then logs serial output for <seconds>.
import sys, time, serial

port, secs = sys.argv[1], float(sys.argv[2])
cmds = sys.argv[3] if len(sys.argv) > 3 else ""
out = sys.argv[4] if len(sys.argv) > 4 else None

s = serial.Serial(port, 115200, timeout=0.1)
time.sleep(0.3)
s.reset_input_buffer()
for c in cmds:
    s.write(c.encode())
    time.sleep(0.05)

lines = []
end = time.time() + secs
buf = b""
while time.time() < end:
    buf += s.read(4096)
    *done, buf = buf.split(b"\n")
    lines += [l.decode(errors="replace").rstrip("\r") for l in done]
s.close()

text = "\n".join(lines)
if out:
    open(out, "w").write(text)
    print(f"{len(lines)} lines -> {out}")
else:
    print(text)
