from __future__ import annotations

import json
import socket
import struct


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        part = sock.recv(n - len(buf))
        if not part:
            raise ConnectionError("对端关闭连接")
        buf += part
    return buf


def send_msg(sock: socket.socket, header: dict, blobs: list[bytes] = ()) -> None:
    h = json.dumps(header).encode()
    sock.sendall(struct.pack("!I", len(h)) + h + b"".join(blobs))


def recv_msg(sock: socket.socket) -> tuple[dict, list[bytes]]:
    (hlen,) = struct.unpack("!I", _recv_exact(sock, 4))
    header = json.loads(_recv_exact(sock, hlen))
    blobs = [_recv_exact(sock, n) for n in header.get("jpeg_lens", [])]
    return header, blobs
