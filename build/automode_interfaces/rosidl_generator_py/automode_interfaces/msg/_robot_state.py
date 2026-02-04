# generated from rosidl_generator_py/resource/_idl.py.em
# with input from automode_interfaces:msg/RobotState.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RobotState(type):
    """Metaclass of message 'RobotState'."""

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
            module = import_type_support('automode_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'automode_interfaces.msg.RobotState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__robot_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__robot_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__robot_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__robot_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__robot_state

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RobotState(metaclass=Metaclass_RobotState):
    """Message class 'RobotState'."""

    __slots__ = [
        '_robot_id',
        '_stamp',
        '_floor_color',
        '_proximity_magnitude',
        '_proximity_angle',
        '_light_magnitude',
        '_light_angle',
        '_target_magnitude',
        '_target_position',
        '_attraction_angle',
        '_neighbour_count',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'robot_id': 'uint32',
        'stamp': 'builtin_interfaces/Time',
        'floor_color': 'string',
        'proximity_magnitude': 'double',
        'proximity_angle': 'double',
        'light_magnitude': 'double',
        'light_angle': 'double',
        'target_magnitude': 'double',
        'target_position': 'double',
        'attraction_angle': 'double',
        'neighbour_count': 'uint32',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.robot_id = kwargs.get('robot_id', int())
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())
        self.floor_color = kwargs.get('floor_color', str())
        self.proximity_magnitude = kwargs.get('proximity_magnitude', float())
        self.proximity_angle = kwargs.get('proximity_angle', float())
        self.light_magnitude = kwargs.get('light_magnitude', float())
        self.light_angle = kwargs.get('light_angle', float())
        self.target_magnitude = kwargs.get('target_magnitude', float())
        self.target_position = kwargs.get('target_position', float())
        self.attraction_angle = kwargs.get('attraction_angle', float())
        self.neighbour_count = kwargs.get('neighbour_count', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.robot_id != other.robot_id:
            return False
        if self.stamp != other.stamp:
            return False
        if self.floor_color != other.floor_color:
            return False
        if self.proximity_magnitude != other.proximity_magnitude:
            return False
        if self.proximity_angle != other.proximity_angle:
            return False
        if self.light_magnitude != other.light_magnitude:
            return False
        if self.light_angle != other.light_angle:
            return False
        if self.target_magnitude != other.target_magnitude:
            return False
        if self.target_position != other.target_position:
            return False
        if self.attraction_angle != other.attraction_angle:
            return False
        if self.neighbour_count != other.neighbour_count:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def robot_id(self):
        """Message field 'robot_id'."""
        return self._robot_id

    @robot_id.setter
    def robot_id(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'robot_id' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'robot_id' field must be an unsigned integer in [0, 4294967295]"
        self._robot_id = value

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if self._check_fields:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value

    @builtins.property
    def floor_color(self):
        """Message field 'floor_color'."""
        return self._floor_color

    @floor_color.setter
    def floor_color(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'floor_color' field must be of type 'str'"
        self._floor_color = value

    @builtins.property
    def proximity_magnitude(self):
        """Message field 'proximity_magnitude'."""
        return self._proximity_magnitude

    @proximity_magnitude.setter
    def proximity_magnitude(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'proximity_magnitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'proximity_magnitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._proximity_magnitude = value

    @builtins.property
    def proximity_angle(self):
        """Message field 'proximity_angle'."""
        return self._proximity_angle

    @proximity_angle.setter
    def proximity_angle(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'proximity_angle' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'proximity_angle' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._proximity_angle = value

    @builtins.property
    def light_magnitude(self):
        """Message field 'light_magnitude'."""
        return self._light_magnitude

    @light_magnitude.setter
    def light_magnitude(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'light_magnitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'light_magnitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._light_magnitude = value

    @builtins.property
    def light_angle(self):
        """Message field 'light_angle'."""
        return self._light_angle

    @light_angle.setter
    def light_angle(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'light_angle' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'light_angle' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._light_angle = value

    @builtins.property
    def target_magnitude(self):
        """Message field 'target_magnitude'."""
        return self._target_magnitude

    @target_magnitude.setter
    def target_magnitude(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'target_magnitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'target_magnitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._target_magnitude = value

    @builtins.property
    def target_position(self):
        """Message field 'target_position'."""
        return self._target_position

    @target_position.setter
    def target_position(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'target_position' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'target_position' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._target_position = value

    @builtins.property
    def attraction_angle(self):
        """Message field 'attraction_angle'."""
        return self._attraction_angle

    @attraction_angle.setter
    def attraction_angle(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'attraction_angle' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'attraction_angle' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._attraction_angle = value

    @builtins.property
    def neighbour_count(self):
        """Message field 'neighbour_count'."""
        return self._neighbour_count

    @neighbour_count.setter
    def neighbour_count(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'neighbour_count' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'neighbour_count' field must be an unsigned integer in [0, 4294967295]"
        self._neighbour_count = value
