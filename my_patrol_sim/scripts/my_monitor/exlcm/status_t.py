"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class status_t(object):
    __slots__ = ["robot_id", "frame_id", "x_robot", "y_robot", "th_robot"]

    __typenames__ = ["int32_t", "string", "double", "double", "double"]

    __dimensions__ = [None, None, None, None, None]

    def __init__(self):
        self.robot_id = 0
        self.frame_id = ""
        self.x_robot = 0.0
        self.y_robot = 0.0
        self.th_robot = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(status_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">i", self.robot_id))
        __frame_id_encoded = self.frame_id.encode('utf-8')
        buf.write(struct.pack('>I', len(__frame_id_encoded)+1))
        buf.write(__frame_id_encoded)
        buf.write(b"\0")
        buf.write(struct.pack(">ddd", self.x_robot, self.y_robot, self.th_robot))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != status_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return status_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = status_t()
        self.robot_id = struct.unpack(">i", buf.read(4))[0]
        __frame_id_len = struct.unpack('>I', buf.read(4))[0]
        self.frame_id = buf.read(__frame_id_len)[:-1].decode('utf-8', 'replace')
        self.x_robot, self.y_robot, self.th_robot = struct.unpack(">ddd", buf.read(24))
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if status_t in parents: return 0
        tmphash = (0xa61740edee6e40ff) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if status_t._packed_fingerprint is None:
            status_t._packed_fingerprint = struct.pack(">Q", status_t._get_hash_recursive([]))
        return status_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", status_t._get_packed_fingerprint())[0]

