"""Buffered socket output over local stream and datagram sockets."""
import builtins
import importlib.util
from pathlib import Path
import socket
from unittest.mock import patch

import pytest


class LimitedSend:
    def __init__(self, sock, limit):
        self.sock = sock
        self.limit = limit

    def __getattr__(self, name):
        return getattr(self.sock, name)

    def send(self, data):
        return self.sock.send(data[:self.limit])


def load_socket_class(fallback):
    path = Path(__file__).resolve().parents[1] / 'pypilot' / 'bufferedsocket.py'
    spec = importlib.util.spec_from_file_location('bufferedsocket_send_test', path)
    module = importlib.util.module_from_spec(spec)
    original_import = builtins.__import__

    def import_module(name, *args, **kwargs):
        if fallback and name == 'pypilot.linebuffer':
            raise ImportError('exercise Python fallback')
        return original_import(name, *args, **kwargs)

    with patch.object(builtins, '__import__', import_module):
        spec.loader.exec_module(module)
    return module.LineBufferedNonBlockingSocket


@pytest.mark.parametrize('fallback', [False, True])
@pytest.mark.parametrize('limit', [1, 2, 5])
@pytest.mark.parametrize('message', ['profile="été海"\n', 'heading=123\n'])
def test_partial_send_preserves_message(fallback, limit, message):
    cls = load_socket_class(fallback)
    sender, receiver = socket.socketpair()
    receiver.settimeout(1)
    stream = cls(LimitedSend(sender, limit), ('localhost', 1234))
    try:
        stream.write(message)
        for _ in range(len(message.encode()) + 1):
            if not stream.out_buffer:
                break
            stream.flush()
        assert not stream.out_buffer
        assert receiver.recv(1024) == message.encode()
        stream.write('next=1\n')
        while stream.out_buffer:
            stream.flush()
        assert receiver.recv(1024) == b'next=1\n'
    finally:
        stream.close()
        receiver.close()


@pytest.mark.parametrize('fallback', [False, True])
def test_watch_uses_udp_only_after_port_is_set(fallback):
    sender, receiver = socket.socketpair()
    with sender, receiver, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        udp.bind(('127.0.0.1', 0))
        udp.settimeout(1)
        receiver.settimeout(1)
        stream = load_socket_class(fallback)(sender, ('127.0.0.1', 1234))
        try:
            stream.write('heading=1\n', True)
            stream.udp_port = udp.getsockname()[1]
            stream.write('heading=2\n', True)
            stream.flush()
            assert receiver.recv(1024) == b'heading=1\n'
            assert udp.recv(1024) == b'heading=2\n'
        finally:
            stream.close()
