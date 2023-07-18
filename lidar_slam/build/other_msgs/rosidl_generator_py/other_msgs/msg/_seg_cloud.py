# generated from rosidl_generator_py/resource/_idl.py.em
# with input from other_msgs:msg/SegCloud.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'seg_ring_str_ind'
# Member 'seg_ring_end_ind'
# Member 'seg_range'
# Member 'is_ground'
# Member 'seg_cloud_col_ind'
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
        '_seg_ring_str_ind',
        '_seg_ring_end_ind',
        '_seg_cloud',
        '_seg_range',
        '_is_ground',
        '_seg_cloud_col_ind',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'seg_ring_str_ind': 'sequence<int32>',
        'seg_ring_end_ind': 'sequence<int32>',
        'seg_cloud': 'sequence<other_msgs/Point>',
        'seg_range': 'sequence<float>',
        'is_ground': 'sequence<int32>',
        'seg_cloud_col_ind': 'sequence<int32>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['other_msgs', 'msg'], 'Point')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.seg_ring_str_ind = array.array('i', kwargs.get('seg_ring_str_ind', []))
        self.seg_ring_end_ind = array.array('i', kwargs.get('seg_ring_end_ind', []))
        self.seg_cloud = kwargs.get('seg_cloud', [])
        self.seg_range = array.array('f', kwargs.get('seg_range', []))
        self.is_ground = array.array('i', kwargs.get('is_ground', []))
        self.seg_cloud_col_ind = array.array('i', kwargs.get('seg_cloud_col_ind', []))

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
        if self.seg_ring_str_ind != other.seg_ring_str_ind:
            return False
        if self.seg_ring_end_ind != other.seg_ring_end_ind:
            return False
        if self.seg_cloud != other.seg_cloud:
            return False
        if self.seg_range != other.seg_range:
            return False
        if self.is_ground != other.is_ground:
            return False
        if self.seg_cloud_col_ind != other.seg_cloud_col_ind:
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
    def seg_ring_str_ind(self):
        """Message field 'seg_ring_str_ind'."""
        return self._seg_ring_str_ind

    @seg_ring_str_ind.setter
    def seg_ring_str_ind(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'seg_ring_str_ind' array.array() must have the type code of 'i'"
            self._seg_ring_str_ind = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'seg_ring_str_ind' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._seg_ring_str_ind = array.array('i', value)

    @builtins.property
    def seg_ring_end_ind(self):
        """Message field 'seg_ring_end_ind'."""
        return self._seg_ring_end_ind

    @seg_ring_end_ind.setter
    def seg_ring_end_ind(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'seg_ring_end_ind' array.array() must have the type code of 'i'"
            self._seg_ring_end_ind = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'seg_ring_end_ind' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._seg_ring_end_ind = array.array('i', value)

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
    def is_ground(self):
        """Message field 'is_ground'."""
        return self._is_ground

    @is_ground.setter
    def is_ground(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'is_ground' array.array() must have the type code of 'i'"
            self._is_ground = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'is_ground' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._is_ground = array.array('i', value)

    @builtins.property
    def seg_cloud_col_ind(self):
        """Message field 'seg_cloud_col_ind'."""
        return self._seg_cloud_col_ind

    @seg_cloud_col_ind.setter
    def seg_cloud_col_ind(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'seg_cloud_col_ind' array.array() must have the type code of 'i'"
            self._seg_cloud_col_ind = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'seg_cloud_col_ind' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._seg_cloud_col_ind = array.array('i', value)
