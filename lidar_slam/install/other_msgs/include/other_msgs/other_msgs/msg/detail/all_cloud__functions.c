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
// Member `corner_sharp`
// Member `corner_less_sharp`
// Member `surf_flat`
// Member `surf_less_flat`
// Member `full_point_res`
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
  // corner_sharp
  if (!other_msgs__msg__Point__Sequence__init(&msg->corner_sharp, 0)) {
    other_msgs__msg__AllCloud__fini(msg);
    return false;
  }
  // corner_less_sharp
  if (!other_msgs__msg__Point__Sequence__init(&msg->corner_less_sharp, 0)) {
    other_msgs__msg__AllCloud__fini(msg);
    return false;
  }
  // surf_flat
  if (!other_msgs__msg__Point__Sequence__init(&msg->surf_flat, 0)) {
    other_msgs__msg__AllCloud__fini(msg);
    return false;
  }
  // surf_less_flat
  if (!other_msgs__msg__Point__Sequence__init(&msg->surf_less_flat, 0)) {
    other_msgs__msg__AllCloud__fini(msg);
    return false;
  }
  // full_point_res
  if (!other_msgs__msg__Point__Sequence__init(&msg->full_point_res, 0)) {
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
  // corner_sharp
  other_msgs__msg__Point__Sequence__fini(&msg->corner_sharp);
  // corner_less_sharp
  other_msgs__msg__Point__Sequence__fini(&msg->corner_less_sharp);
  // surf_flat
  other_msgs__msg__Point__Sequence__fini(&msg->surf_flat);
  // surf_less_flat
  other_msgs__msg__Point__Sequence__fini(&msg->surf_less_flat);
  // full_point_res
  other_msgs__msg__Point__Sequence__fini(&msg->full_point_res);
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
  // corner_sharp
  if (!other_msgs__msg__Point__Sequence__are_equal(
      &(lhs->corner_sharp), &(rhs->corner_sharp)))
  {
    return false;
  }
  // corner_less_sharp
  if (!other_msgs__msg__Point__Sequence__are_equal(
      &(lhs->corner_less_sharp), &(rhs->corner_less_sharp)))
  {
    return false;
  }
  // surf_flat
  if (!other_msgs__msg__Point__Sequence__are_equal(
      &(lhs->surf_flat), &(rhs->surf_flat)))
  {
    return false;
  }
  // surf_less_flat
  if (!other_msgs__msg__Point__Sequence__are_equal(
      &(lhs->surf_less_flat), &(rhs->surf_less_flat)))
  {
    return false;
  }
  // full_point_res
  if (!other_msgs__msg__Point__Sequence__are_equal(
      &(lhs->full_point_res), &(rhs->full_point_res)))
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
  // corner_sharp
  if (!other_msgs__msg__Point__Sequence__copy(
      &(input->corner_sharp), &(output->corner_sharp)))
  {
    return false;
  }
  // corner_less_sharp
  if (!other_msgs__msg__Point__Sequence__copy(
      &(input->corner_less_sharp), &(output->corner_less_sharp)))
  {
    return false;
  }
  // surf_flat
  if (!other_msgs__msg__Point__Sequence__copy(
      &(input->surf_flat), &(output->surf_flat)))
  {
    return false;
  }
  // surf_less_flat
  if (!other_msgs__msg__Point__Sequence__copy(
      &(input->surf_less_flat), &(output->surf_less_flat)))
  {
    return false;
  }
  // full_point_res
  if (!other_msgs__msg__Point__Sequence__copy(
      &(input->full_point_res), &(output->full_point_res)))
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
