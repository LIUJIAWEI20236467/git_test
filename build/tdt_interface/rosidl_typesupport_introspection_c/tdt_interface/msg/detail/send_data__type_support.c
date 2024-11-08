// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from tdt_interface:msg/SendData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "tdt_interface/msg/detail/send_data__rosidl_typesupport_introspection_c.h"
#include "tdt_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "tdt_interface/msg/detail/send_data__functions.h"
#include "tdt_interface/msg/detail/send_data__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void tdt_interface__msg__SendData__rosidl_typesupport_introspection_c__SendData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tdt_interface__msg__SendData__init(message_memory);
}

void tdt_interface__msg__SendData__rosidl_typesupport_introspection_c__SendData_fini_function(void * message_memory)
{
  tdt_interface__msg__SendData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember tdt_interface__msg__SendData__rosidl_typesupport_introspection_c__SendData_message_member_array[3] = {
  {
    "yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__SendData, yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pitch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__SendData, pitch),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "if_shoot",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__SendData, if_shoot),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tdt_interface__msg__SendData__rosidl_typesupport_introspection_c__SendData_message_members = {
  "tdt_interface__msg",  // message namespace
  "SendData",  // message name
  3,  // number of fields
  sizeof(tdt_interface__msg__SendData),
  tdt_interface__msg__SendData__rosidl_typesupport_introspection_c__SendData_message_member_array,  // message members
  tdt_interface__msg__SendData__rosidl_typesupport_introspection_c__SendData_init_function,  // function to initialize message memory (memory has to be allocated)
  tdt_interface__msg__SendData__rosidl_typesupport_introspection_c__SendData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tdt_interface__msg__SendData__rosidl_typesupport_introspection_c__SendData_message_type_support_handle = {
  0,
  &tdt_interface__msg__SendData__rosidl_typesupport_introspection_c__SendData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tdt_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tdt_interface, msg, SendData)() {
  if (!tdt_interface__msg__SendData__rosidl_typesupport_introspection_c__SendData_message_type_support_handle.typesupport_identifier) {
    tdt_interface__msg__SendData__rosidl_typesupport_introspection_c__SendData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tdt_interface__msg__SendData__rosidl_typesupport_introspection_c__SendData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
