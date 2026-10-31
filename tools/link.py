"""The sender's link to grblHAL, over USB serial or a WebSocket.

    usb:/tmp/flexihal-usb      serial device (pyserial)
    ws://localhost:8080        WebSocket (a minimal RFC 6455 client, standard library only)

Both have the same small interface: send(bytes), readline(timeout) and close().
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
            for separator in (b"\n", b"\r"):
                end = self._buf.find(separator)
                if end >= 0:
                    line, self._buf = self._buf[:end], self._buf[end + 1:]
                    line = line.strip(b"\r\n")
                    if line:
                        return line.decode("ascii", "replace")
                    break
            else:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._buf += self._recv(min(remaining, 0.05))
                continue

    def lines_until(self, done, timeout):
        """Read lines until done(line) is true, and return all the lines read."""
        lines = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.readline(deadline - time.monotonic())
            if line is None:
                break
            lines.append(line)
            if done(line):
                break
        return lines

    def command(self, line, timeout=2.0):
        """Send one line and return the replies, up to and including the ok or error."""
        self.send((line + "\n").encode("ascii"))
        return self.lines_until(lambda reply: reply == "ok" or reply.startswith("error"), timeout)


class SerialLink(Link):
    def __init__(self, path):
        super().__init__()
        import serial  # pyserial
        self.port = serial.Serial(path, 115200, timeout=0)
        try:
            self.port.dtr = True
        except OSError:
            pass  # a pseudo-terminal has no modem lines, so having it open counts as DTR

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
        # Unwrap every complete frame received so far and keep the text and binary payloads.
        data = b""
        while len(self._raw) >= 2:
            opcode, length, start = self._raw[0] & 0x0F, self._raw[1] & 0x7F, 2
            if length == 126:
                if len(self._raw) < 4:
                    break
                length, start = struct.unpack(">H", self._raw[2:4])[0], 4
            elif length == 127:
                if len(self._raw) < 10:
                    break
                length, start = struct.unpack(">Q", self._raw[2:10])[0], 10
            if len(self._raw) < start + length:
                break
            payload, self._raw = self._raw[start:start + length], self._raw[start + length:]
            if opcode in (0x1, 0x2):
                data += payload
        return data

    def send(self, data):
        # Client frames have to be masked.
        mask = os.urandom(4)
        length = len(data)
        if length < 126:
            header = struct.pack("BB", 0x81, 0x80 | length)
        elif length < 65536:
            header = struct.pack(">BBH", 0x81, 0x80 | 126, length)
        else:
            header = struct.pack(">BBQ", 0x81, 0x80 | 127, length)
        body = bytes(byte ^ mask[i % 4] for i, byte in enumerate(data))
        self.sock.sendall(header + mask + body)

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
