"""Sender-side link to grblHAL over USB serial or WebSocket.

    usb:/tmp/flexihal-usb      serial device (pyserial)
    ws://localhost:8080        WebSocket (minimal RFC 6455 client, stdlib only)

Both give the same small interface: send(bytes), readline(timeout), close().
"""

import base64
import os
import socket
import struct
import time


class Link:
    def __init__(self):
        self._buf = b""

    def _recv(self, timeout):
        raise NotImplementedError

    def send(self, data):
        raise NotImplementedError

    def close(self):
        raise NotImplementedError

    def readline(self, timeout=1.0):
        """Next line without its line ending, or None on timeout."""
        deadline = time.monotonic() + timeout
        while True:
            for sep in (b"\n", b"\r"):
                i = self._buf.find(sep)
                if i >= 0:
                    line, self._buf = self._buf[:i], self._buf[i + 1:]
                    line = line.strip(b"\r\n")
                    if line:
                        return line.decode("ascii", "replace")
                    break
            else:
                left = deadline - time.monotonic()
                if left <= 0:
                    return None
                self._buf += self._recv(min(left, 0.05))
                continue

    def lines_until(self, done, timeout):
        """Read lines until done(line) is true; returns all lines read."""
        got = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.readline(deadline - time.monotonic())
            if line is None:
                break
            got.append(line)
            if done(line):
                break
        return got

    def command(self, line, timeout=2.0):
        """Send one line and return the replies up to and including ok/error."""
        self.send((line + "\n").encode("ascii"))
        return self.lines_until(lambda l: l == "ok" or l.startswith("error"), timeout)


class SerialLink(Link):
    def __init__(self, path):
        super().__init__()
        import serial  # pyserial
        self.port = serial.Serial(path, 115200, timeout=0)
        try:
            self.port.dtr = True
        except OSError:
            pass  # pseudo-terminal: no modem lines; having it open is the "DTR"

    def _recv(self, timeout):
        end = time.monotonic() + timeout
        while True:
            data = self.port.read(4096)
            if data or time.monotonic() >= end:
                return data
            time.sleep(0.002)

    def send(self, data):
        self.port.write(data)

    def close(self):
        self.port.close()


class WebSocketLink(Link):
    def __init__(self, url):
        super().__init__()
        rest = url[len("ws://"):]
        hostport, _, path = rest.partition("/")
        host, _, port = hostport.partition(":")
        self.sock = socket.create_connection((host, int(port or 80)), timeout=5)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall((
            f"GET /{path} HTTP/1.1\r\nHost: {hostport}\r\nUpgrade: websocket\r\n"
            f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n").encode())
        reply = b""
        while b"\r\n\r\n" not in reply:
            chunk = self.sock.recv(1024)
            if not chunk:
                raise ConnectionError("WebSocket handshake closed")
            reply += chunk
        if b" 101 " not in reply.split(b"\r\n", 1)[0]:
            raise ConnectionError(reply.split(b"\r\n", 1)[0].decode())
        self._raw = reply.split(b"\r\n\r\n", 1)[1]

    def _recv(self, timeout):
        self.sock.settimeout(timeout)
        try:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("WebSocket closed")
            self._raw += chunk
        except socket.timeout:
            pass
        out = b""
        while len(self._raw) >= 2:
            b1, b2 = self._raw[0], self._raw[1]
            n, off = b2 & 0x7F, 2
            if n == 126:
                if len(self._raw) < 4:
                    break
                n, off = struct.unpack(">H", self._raw[2:4])[0], 4
            elif n == 127:
                if len(self._raw) < 10:
                    break
                n, off = struct.unpack(">Q", self._raw[2:10])[0], 10
            if len(self._raw) < off + n:
                break
            payload, self._raw = self._raw[off:off + n], self._raw[off + n:]
            if b1 & 0x0F in (0x1, 0x2):
                out += payload
        return out

    def send(self, data):
        mask = os.urandom(4)
        n = len(data)
        if n < 126:
            head = struct.pack("BB", 0x81, 0x80 | n)
        elif n < 65536:
            head = struct.pack(">BBH", 0x81, 0x80 | 126, n)
        else:
            head = struct.pack(">BBQ", 0x81, 0x80 | 127, n)
        body = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
        self.sock.sendall(head + mask + body)

    def close(self):
        try:
            self.sock.sendall(struct.pack("BB", 0x88, 0x80) + os.urandom(4))
        except OSError:
            pass
        self.sock.close()


def open_link(target):
    if target.startswith("ws://"):
        return WebSocketLink(target)
    if target.startswith("usb:"):
        return SerialLink(target[len("usb:"):])
    raise ValueError(f"unknown link {target!r}: use usb:<device> or ws://host:port")
