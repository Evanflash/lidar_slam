# generated from rosidl_generator_py/resource/_idl.py.em
# with input from other_msgs:msg/AllCloud.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'trans_form'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_AllCloud(type):
    """Metaclass of message 'AllCloud'."""

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
                'other_msgs.msg.AllCloud')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__all_cloud
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__all_cloud
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__all_cloud
            cls._TYPE_SUPPORT = module.type_support_msg__msg__all_cloud
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__all_cloud

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


class AllCloud(metaclass=Metaclass_AllCloud):
    """Message class 'AllCloud'."""

    __slots__ = [
        '_header',
        '_trans_form',
        '_corner_sharp',
        '_corner_less_sharp',
        '_surf_less_flat',
        '_ground_flat',
        '_ground_less_flat',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'trans_form': 'sequence<float>',
        'corner_sharp': 'sequence<other_msgs/Point>',
        'corner_less_sharp': 'sequence<other_msgs/Point>',
        'surf_less_flat': 'sequence<other_msgs/Point>',
        'ground_flat': 'sequence<other_msgs/Point>',
        'ground_less_flat': 'sequence<other_msgs/Point>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['other_msgs', 'msg'], 'Point')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['other_msgs', 'msg'], 'Point')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['other_msgs', 'msg'], 'Point')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['other_msgs', 'msg'], 'Point')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['other_msgs', 'msg'], 'Point')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.trans_form = array.array('f', kwargs.get('trans_form', []))
        self.corner_sharp = kwargs.get('corner_sharp', [])
        self.corner_less_sharp = kwargs.get('corner_less_sharp', [])
        self.surf_less_flat = kwargs.get('surf_less_flat', [])
        self.ground_flat = kwargs.get('ground_flat', [])
        self.ground_less_flat = kwargs.get('ground_less_flat', [])

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
        if self.trans_form != other.trans_form:
            return False
        if self.corner_sharp != other.corner_sharp:
            return False
        if self.corner_less_sharp != other.corner_less_sharp:
            return False
        if self.surf_less_flat != other.surf_less_flat:
            return False
        if self.ground_flat != other.ground_flat:
            return False
        if self.ground_less_flat != other.ground_less_flat:
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
    def trans_form(self):
        """Message field 'trans_form'."""
        return self._trans_form

    @trans_form.setter
    def trans_form(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'trans_form' array.array() must have the type code of 'f'"
            self._trans_form = value
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
                "The 'trans_form' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._trans_form = array.array('f', value)

    @builtins.property
    def corner_sharp(self):
        """Message field 'corner_sharp'."""
        return self._corner_sharp

    @corner_sharp.setter
    def corner_sharp(self, value):
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
                "The 'corner_sharp' field must be a set or sequence and each value of type 'Point'"
        self._corner_sharp = value

    @builtins.property
    def corner_less_sharp(self):
        """Message field 'corner_less_sharp'."""
        return self._corner_less_sharp

    @corner_less_sharp.setter
    def corner_less_sharp(self, value):
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
                "The 'corner_less_sharp' field must be a set or sequence and each value of type 'Point'"
        self._corner_less_sharp = value

    @builtins.property
    def surf_less_flat(self):
        """Message field 'surf_less_flat'."""
        return self._surf_less_flat

    @surf_less_flat.setter
    def surf_less_flat(self, value):
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
                "The 'surf_less_flat' field must be a set or sequence and each value of type 'Point'"
        self._surf_less_flat = value

    @builtins.property
    def ground_flat(self):
        """Message field 'ground_flat'."""
        return self._ground_flat

    @ground_flat.setter
    def ground_flat(self, value):
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
                "The 'ground_flat' field must be a set or sequence and each value of type 'Point'"
        self._ground_flat = value

    @builtins.property
    def ground_less_flat(self):
        """Message field 'ground_less_flat'."""
        return self._ground_less_flat

    @ground_less_flat.setter
    def ground_less_flat(self, value):
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
                "The 'ground_less_flat' field must be a set or sequence and each value of type 'Point'"
        self._ground_less_flat = value
