def packet(sid, instruction, params=()):
    body = bytes([sid, len(params) + 2, instruction, *params])
    return b'\xff\xff' + body + bytes([(~sum(body)) & 255])


def word(value):
    return [value & 255, value >> 8]


