# generated from rosidl_generator_py/resource/_idl.py.em
# with input from other_msgs:msg/SegCloud.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'seg_range'
# Member 'ground_range'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SegCloud(type):
    """Metaclass of message 'SegCloud'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('other_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'other_msgs.msg.SegCloud')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__seg_cloud
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__seg_cloud
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__seg_cloud
            cls._TYPE_SUPPORT = module.type_support_msg__msg__seg_cloud
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__seg_cloud

            from other_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SegCloud(metaclass=Metaclass_SegCloud):
    """Message class 'SegCloud'."""

    __slots__ = [
        '_header',
        '_seg_cloud',
        '_seg_range',
        '_ground_cloud',
        '_ground_range',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'seg_cloud': 'sequence<other_msgs/Point>',
        'seg_range': 'sequence<float>',
        'ground_cloud': 'sequence<other_msgs/Point>',
        'ground_range': 'sequence<float>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['other_msgs', 'msg'], 'Point')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['other_msgs', 'msg'], 'Point')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.seg_cloud = kwargs.get('seg_cloud', [])
        self.seg_range = array.array('f', kwargs.get('seg_range', []))
        self.ground_cloud = kwargs.get('ground_cloud', [])
        self.ground_range = array.array('f', kwargs.get('ground_range', []))

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.seg_cloud != other.seg_cloud:
            return False
        if self.seg_range != other.seg_range:
            return False
        if self.ground_cloud != other.ground_cloud:
            return False
        if self.ground_range != other.ground_range:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def seg_cloud(self):
        """Message field 'seg_cloud'."""
        return self._seg_cloud

    @seg_cloud.setter
    def seg_cloud(self, value):
        if __debug__:
            from other_msgs.msg import Point
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, Point) for v in value) and
                 True), \
                "The 'seg_cloud' field must be a set or sequence and each value of type 'Point'"
        self._seg_cloud = value

    @builtins.property
    def seg_range(self):
        """Message field 'seg_range'."""
        return self._seg_range

    @seg_range.setter
    def seg_range(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'seg_range' array.array() must have the type code of 'f'"
            self._seg_range = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'seg_range' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._seg_range = array.array('f', value)

    @builtins.property
    def ground_cloud(self):
        """Message field 'ground_cloud'."""
        return self._ground_cloud

    @ground_cloud.setter
    def ground_cloud(self, value):
        if __debug__:
            from other_msgs.msg import Point
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, Point) for v in value) and
                 True), \
                "The 'ground_cloud' field must be a set or sequence and each value of type 'Point'"
        self._ground_cloud = value

    @builtins.property
    def ground_range(self):
        """Message field 'ground_range'."""
        return self._ground_range

    @ground_range.setter
    def ground_range(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'ground_range' array.array() must have the type code of 'f'"
            self._ground_range = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'ground_range' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._ground_range = array.array('f', value)
