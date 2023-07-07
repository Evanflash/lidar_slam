// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from other_msgs:msg/AllCloud.idl
// generated code does not contain a copyright notice

#ifndef OTHER_MSGS__MSG__DETAIL__ALL_CLOUD__FUNCTIONS_H_
#define OTHER_MSGS__MSG__DETAIL__ALL_CLOUD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "other_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "other_msgs/msg/detail/all_cloud__struct.h"

/// Initialize msg/AllCloud message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * other_msgs__msg__AllCloud
 * )) before or use
 * other_msgs__msg__AllCloud__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_other_msgs
bool
other_msgs__msg__AllCloud__init(other_msgs__msg__AllCloud * msg);

/// Finalize msg/AllCloud message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_other_msgs
void
other_msgs__msg__AllCloud__fini(other_msgs__msg__AllCloud * msg);

/// Create msg/AllCloud message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * other_msgs__msg__AllCloud__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_other_msgs
other_msgs__msg__AllCloud *
other_msgs__msg__AllCloud__create();

/// Destroy msg/AllCloud message.
/**
 * It calls
 * other_msgs__msg__AllCloud__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_other_msgs
void
other_msgs__msg__AllCloud__destroy(other_msgs__msg__AllCloud * msg);

/// Check for msg/AllCloud message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_other_msgs
bool
other_msgs__msg__AllCloud__are_equal(const other_msgs__msg__AllCloud * lhs, const other_msgs__msg__AllCloud * rhs);

/// Copy a msg/AllCloud message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_other_msgs
bool
other_msgs__msg__AllCloud__copy(
  const other_msgs__msg__AllCloud * input,
  other_msgs__msg__AllCloud * output);

/// Initialize array of msg/AllCloud messages.
/**
 * It allocates the memory for the number of elements and calls
 * other_msgs__msg__AllCloud__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_other_msgs
bool
other_msgs__msg__AllCloud__Sequence__init(other_msgs__msg__AllCloud__Sequence * array, size_t size);

/// Finalize array of msg/AllCloud messages.
/**
 * It calls
 * other_msgs__msg__AllCloud__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_other_msgs
void
other_msgs__msg__AllCloud__Sequence__fini(other_msgs__msg__AllCloud__Sequence * array);

/// Create array of msg/AllCloud messages.
/**
 * It allocates the memory for the array and calls
 * other_msgs__msg__AllCloud__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_other_msgs
other_msgs__msg__AllCloud__Sequence *
other_msgs__msg__AllCloud__Sequence__create(size_t size);

/// Destroy array of msg/AllCloud messages.
/**
 * It calls
 * other_msgs__msg__AllCloud__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_other_msgs
void
other_msgs__msg__AllCloud__Sequence__destroy(other_msgs__msg__AllCloud__Sequence * array);

/// Check for msg/AllCloud message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_other_msgs
bool
other_msgs__msg__AllCloud__Sequence__are_equal(const other_msgs__msg__AllCloud__Sequence * lhs, const other_msgs__msg__AllCloud__Sequence * rhs);

/// Copy an array of msg/AllCloud messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_other_msgs
bool
other_msgs__msg__AllCloud__Sequence__copy(
  const other_msgs__msg__AllCloud__Sequence * input,
  other_msgs__msg__AllCloud__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // OTHER_MSGS__MSG__DETAIL__ALL_CLOUD__FUNCTIONS_H_
