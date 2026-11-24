"""Real local sockets and transient-error wrappers; no network service needed."""
import errno
import socket

import pytest
from test_bufferedsocket_send import load_socket_class


@pytest.mark.parametrize('fallback', [False, True])
def test_full_socket_keeps_pending_bytes_for_retry(fallback):
    sender, receiver = socket.socketpair()
    sender.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)
    receiver.setblocking(False)
    stream = load_socket_class(fallback)(sender, ('localhost', 1234))
    try:
        while True:
            try:
                sender.send(b'x' * 4096)
            except BlockingIOError:
                break
        stream.write('heading=123\n')
        stream.flush()
        assert sender.fileno() >= 0
        assert stream.out_buffer == b'heading=123\n'
        while True:
            try:
                receiver.recv(65536)
            except BlockingIOError:
                break
        stream.flush()
        assert not stream.out_buffer
        assert receiver.recv(1024) == b'heading=123\n'
    finally:
        stream.close()
        receiver.close()


@pytest.mark.parametrize('fallback', [False, True])
def test_send_can_retry_after_eagain(fallback):
    sender, receiver = socket.socketpair()
    receiver.settimeout(1)

    class Once:
        def __init__(self):
            self.first = True

        def __getattr__(self, name):
            return getattr(sender, name)

        def send(self, data):
            if self.first:
                self.first = False
                raise BlockingIOError(errno.EAGAIN, 'retry')
            return sender.send(data)

    stream = load_socket_class(fallback)(Once(), ('localhost', 1234))
    try:
        stream.write('heading=0\n')
        stream.flush()
        assert sender.fileno() >= 0
        assert stream.out_buffer == b'heading=0\n'
        stream.flush()
        assert receiver.recv(1024) == b'heading=0\n'
        assert not stream.out_buffer
    finally:
        stream.close()
        receiver.close()


@pytest.mark.parametrize('fallback', [False, True])
def test_closed_peer_still_closes_connection(fallback):
    sender, receiver = socket.socketpair()
    stream = load_socket_class(fallback)(sender, ('localhost', 1234))
    receiver.close()
    try:
        stream.write('heading=0\n')
        stream.flush()
        assert sender.fileno() == -1
    finally:
        stream.close()
