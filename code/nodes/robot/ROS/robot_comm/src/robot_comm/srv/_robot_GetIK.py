"""autogenerated by genpy from robot_comm/robot_GetIKRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class robot_GetIKRequest(genpy.Message):
  _md5sum = "256392fa17e6514709500a65ddaf30e9"
  _type = "robot_comm/robot_GetIKRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """

float64 x
float64 y
float64 z
float64 q0
float64 qx
float64 qy
float64 qz


"""
  __slots__ = ['x','y','z','q0','qx','qy','qz']
  _slot_types = ['float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       x,y,z,q0,qx,qy,qz

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(robot_GetIKRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.z is None:
        self.z = 0.
      if self.q0 is None:
        self.q0 = 0.
      if self.qx is None:
        self.qx = 0.
      if self.qy is None:
        self.qy = 0.
      if self.qz is None:
        self.qz = 0.
    else:
      self.x = 0.
      self.y = 0.
      self.z = 0.
      self.q0 = 0.
      self.qx = 0.
      self.qy = 0.
      self.qz = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_7d.pack(_x.x, _x.y, _x.z, _x.q0, _x.qx, _x.qy, _x.qz))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 56
      (_x.x, _x.y, _x.z, _x.q0, _x.qx, _x.qy, _x.qz,) = _struct_7d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_7d.pack(_x.x, _x.y, _x.z, _x.q0, _x.qx, _x.qy, _x.qz))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 56
      (_x.x, _x.y, _x.z, _x.q0, _x.qx, _x.qy, _x.qz,) = _struct_7d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_7d = struct.Struct("<7d")
"""autogenerated by genpy from robot_comm/robot_GetIKResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class robot_GetIKResponse(genpy.Message):
  _md5sum = "927b82d08077d046362055bd87d33b65"
  _type = "robot_comm/robot_GetIKResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 j1
float64 j2
float64 j3
float64 j4
float64 j5
float64 j6
int64 ret
string msg


"""
  __slots__ = ['j1','j2','j3','j4','j5','j6','ret','msg']
  _slot_types = ['float64','float64','float64','float64','float64','float64','int64','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       j1,j2,j3,j4,j5,j6,ret,msg

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(robot_GetIKResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.j1 is None:
        self.j1 = 0.
      if self.j2 is None:
        self.j2 = 0.
      if self.j3 is None:
        self.j3 = 0.
      if self.j4 is None:
        self.j4 = 0.
      if self.j5 is None:
        self.j5 = 0.
      if self.j6 is None:
        self.j6 = 0.
      if self.ret is None:
        self.ret = 0
      if self.msg is None:
        self.msg = ''
    else:
      self.j1 = 0.
      self.j2 = 0.
      self.j3 = 0.
      self.j4 = 0.
      self.j5 = 0.
      self.j6 = 0.
      self.ret = 0
      self.msg = ''

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_6dq.pack(_x.j1, _x.j2, _x.j3, _x.j4, _x.j5, _x.j6, _x.ret))
      _x = self.msg
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 56
      (_x.j1, _x.j2, _x.j3, _x.j4, _x.j5, _x.j6, _x.ret,) = _struct_6dq.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.msg = str[start:end].decode('utf-8')
      else:
        self.msg = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_6dq.pack(_x.j1, _x.j2, _x.j3, _x.j4, _x.j5, _x.j6, _x.ret))
      _x = self.msg
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 56
      (_x.j1, _x.j2, _x.j3, _x.j4, _x.j5, _x.j6, _x.ret,) = _struct_6dq.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.msg = str[start:end].decode('utf-8')
      else:
        self.msg = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_6dq = struct.Struct("<6dq")
class robot_GetIK(object):
  _type          = 'robot_comm/robot_GetIK'
  _md5sum = 'e8bf11be2a3edf791e341a71c52178e5'
  _request_class  = robot_GetIKRequest
  _response_class = robot_GetIKResponse
