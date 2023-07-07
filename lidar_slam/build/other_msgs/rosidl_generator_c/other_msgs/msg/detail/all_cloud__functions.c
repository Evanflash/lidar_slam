// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from other_msgs:msg/AllCloud.idl
// generated code does not contain a copyright notice
#include "other_msgs/msg/detail/all_cloud__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `cornersharp`
// Member `cornerlesssharp`
// Member `surfflat`
// Member `surflessflat`
// Member `fullpointres`
#include "other_msgs/msg/detail/point__functions.h"

bool
other_msgs__msg__AllCloud__init(other_msgs__msg__AllCloud * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    other_msgs__msg__AllCloud__fini(msg);
    return false;
  }
  // cornersharp
  if (!other_msgs__msg__Point__Sequence__init(&msg->cornersharp, 0)) {
    other_msgs__msg__AllCloud__fini(msg);
    return false;
  }
  // cornerlesssharp
  if (!other_msgs__msg__Point__Sequence__init(&msg->cornerlesssharp, 0)) {
    other_msgs__msg__AllCloud__fini(msg);
    return false;
  }
  // surfflat
  if (!other_msgs__msg__Point__Sequence__init(&msg->surfflat, 0)) {
    other_msgs__msg__AllCloud__fini(msg);
    return false;
  }
  // surflessflat
  if (!other_msgs__msg__Point__Sequence__init(&msg->surflessflat, 0)) {
    other_msgs__msg__AllCloud__fini(msg);
    return false;
  }
  // fullpointres
  if (!other_msgs__msg__Point__Sequence__init(&msg->fullpointres, 0)) {
    other_msgs__msg__AllCloud__fini(msg);
    return false;
  }
  return true;
}

void
other_msgs__msg__AllCloud__fini(other_msgs__msg__AllCloud * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // cornersharp
  other_msgs__msg__Point__Sequence__fini(&msg->cornersharp);
  // cornerlesssharp
  other_msgs__msg__Point__Sequence__fini(&msg->cornerlesssharp);
  // surfflat
  other_msgs__msg__Point__Sequence__fini(&msg->surfflat);
  // surflessflat
  other_msgs__msg__Point__Sequence__fini(&msg->surflessflat);
  // fullpointres
  other_msgs__msg__Point__Sequence__fini(&msg->fullpointres);
}

bool
other_msgs__msg__AllCloud__are_equal(const other_msgs__msg__AllCloud * lhs, const other_msgs__msg__AllCloud * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // cornersharp
  if (!other_msgs__msg__Point__Sequence__are_equal(
      &(lhs->cornersharp), &(rhs->cornersharp)))
  {
    return false;
  }
  // cornerlesssharp
  if (!other_msgs__msg__Point__Sequence__are_equal(
      &(lhs->cornerlesssharp), &(rhs->cornerlesssharp)))
  {
    return false;
  }
  // surfflat
  if (!other_msgs__msg__Point__Sequence__are_equal(
      &(lhs->surfflat), &(rhs->surfflat)))
  {
    return false;
  }
  // surflessflat
  if (!other_msgs__msg__Point__Sequence__are_equal(
      &(lhs->surflessflat), &(rhs->surflessflat)))
  {
    return false;
  }
  // fullpointres
  if (!other_msgs__msg__Point__Sequence__are_equal(
      &(lhs->fullpointres), &(rhs->fullpointres)))
  {
    return false;
  }
  return true;
}

bool
other_msgs__msg__AllCloud__copy(
  const other_msgs__msg__AllCloud * input,
  other_msgs__msg__AllCloud * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // cornersharp
  if (!other_msgs__msg__Point__Sequence__copy(
      &(input->cornersharp), &(output->cornersharp)))
  {
    return false;
  }
  // cornerlesssharp
  if (!other_msgs__msg__Point__Sequence__copy(
      &(input->cornerlesssharp), &(output->cornerlesssharp)))
  {
    return false;
  }
  // surfflat
  if (!other_msgs__msg__Point__Sequence__copy(
      &(input->surfflat), &(output->surfflat)))
  {
    return false;
  }
  // surflessflat
  if (!other_msgs__msg__Point__Sequence__copy(
      &(input->surflessflat), &(output->surflessflat)))
  {
    return false;
  }
  // fullpointres
  if (!other_msgs__msg__Point__Sequence__copy(
      &(input->fullpointres), &(output->fullpointres)))
  {
    return false;
  }
  return true;
}

other_msgs__msg__AllCloud *
other_msgs__msg__AllCloud__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  other_msgs__msg__AllCloud * msg = (other_msgs__msg__AllCloud *)allocator.allocate(sizeof(other_msgs__msg__AllCloud), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(other_msgs__msg__AllCloud));
  bool success = other_msgs__msg__AllCloud__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
other_msgs__msg__AllCloud__destroy(other_msgs__msg__AllCloud * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    other_msgs__msg__AllCloud__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
other_msgs__msg__AllCloud__Sequence__init(other_msgs__msg__AllCloud__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  other_msgs__msg__AllCloud * data = NULL;

  if (size) {
    data = (other_msgs__msg__AllCloud *)allocator.zero_allocate(size, sizeof(other_msgs__msg__AllCloud), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = other_msgs__msg__AllCloud__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        other_msgs__msg__AllCloud__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
other_msgs__msg__AllCloud__Sequence__fini(other_msgs__msg__AllCloud__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      other_msgs__msg__AllCloud__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

other_msgs__msg__AllCloud__Sequence *
other_msgs__msg__AllCloud__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  other_msgs__msg__AllCloud__Sequence * array = (other_msgs__msg__AllCloud__Sequence *)allocator.allocate(sizeof(other_msgs__msg__AllCloud__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = other_msgs__msg__AllCloud__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
other_msgs__msg__AllCloud__Sequence__destroy(other_msgs__msg__AllCloud__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    other_msgs__msg__AllCloud__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
other_msgs__msg__AllCloud__Sequence__are_equal(const other_msgs__msg__AllCloud__Sequence * lhs, const other_msgs__msg__AllCloud__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!other_msgs__msg__AllCloud__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
other_msgs__msg__AllCloud__Sequence__copy(
  const other_msgs__msg__AllCloud__Sequence * input,
  other_msgs__msg__AllCloud__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(other_msgs__msg__AllCloud);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    other_msgs__msg__AllCloud * data =
      (other_msgs__msg__AllCloud *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!other_msgs__msg__AllCloud__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          other_msgs__msg__AllCloud__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!other_msgs__msg__AllCloud__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
