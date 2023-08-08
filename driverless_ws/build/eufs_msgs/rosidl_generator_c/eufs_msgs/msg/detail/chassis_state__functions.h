// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from eufs_msgs:msg/ChassisState.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__MSG__DETAIL__CHASSIS_STATE__FUNCTIONS_H_
#define EUFS_MSGS__MSG__DETAIL__CHASSIS_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "eufs_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "eufs_msgs/msg/detail/chassis_state__struct.h"

/// Initialize msg/ChassisState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * eufs_msgs__msg__ChassisState
 * )) before or use
 * eufs_msgs__msg__ChassisState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
bool
eufs_msgs__msg__ChassisState__init(eufs_msgs__msg__ChassisState * msg);

/// Finalize msg/ChassisState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
void
eufs_msgs__msg__ChassisState__fini(eufs_msgs__msg__ChassisState * msg);

/// Create msg/ChassisState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * eufs_msgs__msg__ChassisState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
eufs_msgs__msg__ChassisState *
eufs_msgs__msg__ChassisState__create();

/// Destroy msg/ChassisState message.
/**
 * It calls
 * eufs_msgs__msg__ChassisState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
void
eufs_msgs__msg__ChassisState__destroy(eufs_msgs__msg__ChassisState * msg);

/// Check for msg/ChassisState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
bool
eufs_msgs__msg__ChassisState__are_equal(const eufs_msgs__msg__ChassisState * lhs, const eufs_msgs__msg__ChassisState * rhs);

/// Copy a msg/ChassisState message.
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
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
bool
eufs_msgs__msg__ChassisState__copy(
  const eufs_msgs__msg__ChassisState * input,
  eufs_msgs__msg__ChassisState * output);

/// Initialize array of msg/ChassisState messages.
/**
 * It allocates the memory for the number of elements and calls
 * eufs_msgs__msg__ChassisState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
bool
eufs_msgs__msg__ChassisState__Sequence__init(eufs_msgs__msg__ChassisState__Sequence * array, size_t size);

/// Finalize array of msg/ChassisState messages.
/**
 * It calls
 * eufs_msgs__msg__ChassisState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
void
eufs_msgs__msg__ChassisState__Sequence__fini(eufs_msgs__msg__ChassisState__Sequence * array);

/// Create array of msg/ChassisState messages.
/**
 * It allocates the memory for the array and calls
 * eufs_msgs__msg__ChassisState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
eufs_msgs__msg__ChassisState__Sequence *
eufs_msgs__msg__ChassisState__Sequence__create(size_t size);

/// Destroy array of msg/ChassisState messages.
/**
 * It calls
 * eufs_msgs__msg__ChassisState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
void
eufs_msgs__msg__ChassisState__Sequence__destroy(eufs_msgs__msg__ChassisState__Sequence * array);

/// Check for msg/ChassisState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
bool
eufs_msgs__msg__ChassisState__Sequence__are_equal(const eufs_msgs__msg__ChassisState__Sequence * lhs, const eufs_msgs__msg__ChassisState__Sequence * rhs);

/// Copy an array of msg/ChassisState messages.
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
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
bool
eufs_msgs__msg__ChassisState__Sequence__copy(
  const eufs_msgs__msg__ChassisState__Sequence * input,
  eufs_msgs__msg__ChassisState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // EUFS_MSGS__MSG__DETAIL__CHASSIS_STATE__FUNCTIONS_H_
