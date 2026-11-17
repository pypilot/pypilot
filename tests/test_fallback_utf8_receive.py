"""The Python fallback must decode the byte stream, not individual packets."""
import socket

import pytest
from test_bufferedsocket_send import load_socket_class


@pytest.mark.parametrize('text', ['profile="été海🌊"\n', 'heading=123\n'])
@pytest.mark.parametrize('chunk_size', [1, 2, 3, 4096])
def test_split_utf8_is_received_as_one_line(text, chunk_size):
    sender, receiver = socket.socketpair()
    stream = load_socket_class(True)(receiver, ('localhost', 1234))
    encoded = text.encode()
    lines = []
    try:
        for offset in range(0, len(encoded), chunk_size):
            chunk = encoded[offset:offset + chunk_size]
            sender.sendall(chunk)
            assert stream.recvdata() == len(chunk)
            line = stream.readline()
            if line:
                lines.append(line)
        assert lines == [text]
        assert stream.readline() == ''
        sender.close()
        assert stream.recvdata() == 0
    finally:
        sender.close()
        stream.close()
