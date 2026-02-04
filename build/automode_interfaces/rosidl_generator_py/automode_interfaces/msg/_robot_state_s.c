// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from automode_interfaces:msg/RobotState.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "automode_interfaces/msg/detail/robot_state__struct.h"
#include "automode_interfaces/msg/detail/robot_state__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool automode_interfaces__msg__robot_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[48];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("automode_interfaces.msg._robot_state.RobotState", full_classname_dest, 47) == 0);
  }
  automode_interfaces__msg__RobotState * ros_message = _ros_message;
  {  // robot_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "robot_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->robot_id = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // stamp
    PyObject * field = PyObject_GetAttrString(_pymsg, "stamp");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__time__convert_from_py(field, &ros_message->stamp)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // floor_color
    PyObject * field = PyObject_GetAttrString(_pymsg, "floor_color");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->floor_color, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // proximity_magnitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "proximity_magnitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->proximity_magnitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // proximity_angle
    PyObject * field = PyObject_GetAttrString(_pymsg, "proximity_angle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->proximity_angle = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // light_magnitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "light_magnitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->light_magnitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // light_angle
    PyObject * field = PyObject_GetAttrString(_pymsg, "light_angle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->light_angle = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // target_magnitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_magnitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->target_magnitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // target_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->target_position = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // attraction_angle
    PyObject * field = PyObject_GetAttrString(_pymsg, "attraction_angle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->attraction_angle = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // neighbour_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "neighbour_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->neighbour_count = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * automode_interfaces__msg__robot_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RobotState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("automode_interfaces.msg._robot_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RobotState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  automode_interfaces__msg__RobotState * ros_message = (automode_interfaces__msg__RobotState *)raw_ros_message;
  {  // robot_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->robot_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "robot_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stamp
    PyObject * field = NULL;
    field = builtin_interfaces__msg__time__convert_to_py(&ros_message->stamp);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "stamp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // floor_color
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->floor_color.data,
      strlen(ros_message->floor_color.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "floor_color", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // proximity_magnitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->proximity_magnitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "proximity_magnitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // proximity_angle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->proximity_angle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "proximity_angle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // light_magnitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->light_magnitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "light_magnitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // light_angle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->light_angle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "light_angle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_magnitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->target_magnitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_magnitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->target_position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // attraction_angle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->attraction_angle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "attraction_angle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // neighbour_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->neighbour_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "neighbour_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
