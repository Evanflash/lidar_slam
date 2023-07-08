// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from other_msgs:msg/SegCloud.idl
// generated code does not contain a copyright notice
#include "other_msgs/msg/detail/seg_cloud__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `seg_ring_str_ind`
// Member `seg_ring_end_ind`
// Member `seg_range`
// Member `grd_ring_str_ind`
// Member `grd_ring_end_ind`
// Member `ground_range`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `seg_cloud`
// Member `ground_cloud`
#include "other_msgs/msg/detail/point__functions.h"

bool
other_msgs__msg__SegCloud__init(other_msgs__msg__SegCloud * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    other_msgs__msg__SegCloud__fini(msg);
    return false;
  }
  // seg_ring_str_ind
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->seg_ring_str_ind, 0)) {
    other_msgs__msg__SegCloud__fini(msg);
    return false;
  }
  // seg_ring_end_ind
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->seg_ring_end_ind, 0)) {
    other_msgs__msg__SegCloud__fini(msg);
    return false;
  }
  // seg_cloud
  if (!other_msgs__msg__Point__Sequence__init(&msg->seg_cloud, 0)) {
    other_msgs__msg__SegCloud__fini(msg);
    return false;
  }
  // seg_range
  if (!rosidl_runtime_c__float__Sequence__init(&msg->seg_range, 0)) {
    other_msgs__msg__SegCloud__fini(msg);
    return false;
  }
  // grd_ring_str_ind
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->grd_ring_str_ind, 0)) {
    other_msgs__msg__SegCloud__fini(msg);
    return false;
  }
  // grd_ring_end_ind
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->grd_ring_end_ind, 0)) {
    other_msgs__msg__SegCloud__fini(msg);
    return false;
  }
  // ground_cloud
  if (!other_msgs__msg__Point__Sequence__init(&msg->ground_cloud, 0)) {
    other_msgs__msg__SegCloud__fini(msg);
    return false;
  }
  // ground_range
  if (!rosidl_runtime_c__float__Sequence__init(&msg->ground_range, 0)) {
    other_msgs__msg__SegCloud__fini(msg);
    return false;
  }
  return true;
}

void
other_msgs__msg__SegCloud__fini(other_msgs__msg__SegCloud * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // seg_ring_str_ind
  rosidl_runtime_c__int32__Sequence__fini(&msg->seg_ring_str_ind);
  // seg_ring_end_ind
  rosidl_runtime_c__int32__Sequence__fini(&msg->seg_ring_end_ind);
  // seg_cloud
  other_msgs__msg__Point__Sequence__fini(&msg->seg_cloud);
  // seg_range
  rosidl_runtime_c__float__Sequence__fini(&msg->seg_range);
  // grd_ring_str_ind
  rosidl_runtime_c__int32__Sequence__fini(&msg->grd_ring_str_ind);
  // grd_ring_end_ind
  rosidl_runtime_c__int32__Sequence__fini(&msg->grd_ring_end_ind);
  // ground_cloud
  other_msgs__msg__Point__Sequence__fini(&msg->ground_cloud);
  // ground_range
  rosidl_runtime_c__float__Sequence__fini(&msg->ground_range);
}

bool
other_msgs__msg__SegCloud__are_equal(const other_msgs__msg__SegCloud * lhs, const other_msgs__msg__SegCloud * rhs)
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
  // seg_ring_str_ind
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->seg_ring_str_ind), &(rhs->seg_ring_str_ind)))
  {
    return false;
  }
  // seg_ring_end_ind
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->seg_ring_end_ind), &(rhs->seg_ring_end_ind)))
  {
    return false;
  }
  // seg_cloud
  if (!other_msgs__msg__Point__Sequence__are_equal(
      &(lhs->seg_cloud), &(rhs->seg_cloud)))
  {
    return false;
  }
  // seg_range
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->seg_range), &(rhs->seg_range)))
  {
    return false;
  }
  // grd_ring_str_ind
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->grd_ring_str_ind), &(rhs->grd_ring_str_ind)))
  {
    return false;
  }
  // grd_ring_end_ind
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->grd_ring_end_ind), &(rhs->grd_ring_end_ind)))
  {
    return false;
  }
  // ground_cloud
  if (!other_msgs__msg__Point__Sequence__are_equal(
      &(lhs->ground_cloud), &(rhs->ground_cloud)))
  {
    return false;
  }
  // ground_range
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->ground_range), &(rhs->ground_range)))
  {
    return false;
  }
  return true;
}

bool
other_msgs__msg__SegCloud__copy(
  const other_msgs__msg__SegCloud * input,
  other_msgs__msg__SegCloud * output)
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
  // seg_ring_str_ind
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->seg_ring_str_ind), &(output->seg_ring_str_ind)))
  {
    return false;
  }
  // seg_ring_end_ind
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->seg_ring_end_ind), &(output->seg_ring_end_ind)))
  {
    return false;
  }
  // seg_cloud
  if (!other_msgs__msg__Point__Sequence__copy(
      &(input->seg_cloud), &(output->seg_cloud)))
  {
    return false;
  }
  // seg_range
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->seg_range), &(output->seg_range)))
  {
    return false;
  }
  // grd_ring_str_ind
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->grd_ring_str_ind), &(output->grd_ring_str_ind)))
  {
    return false;
  }
  // grd_ring_end_ind
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->grd_ring_end_ind), &(output->grd_ring_end_ind)))
  {
    return false;
  }
  // ground_cloud
  if (!other_msgs__msg__Point__Sequence__copy(
      &(input->ground_cloud), &(output->ground_cloud)))
  {
    return false;
  }
  // ground_range
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->ground_range), &(output->ground_range)))
  {
    return false;
  }
  return true;
}

other_msgs__msg__SegCloud *
other_msgs__msg__SegCloud__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  other_msgs__msg__SegCloud * msg = (other_msgs__msg__SegCloud *)allocator.allocate(sizeof(other_msgs__msg__SegCloud), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(other_msgs__msg__SegCloud));
  bool success = other_msgs__msg__SegCloud__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
other_msgs__msg__SegCloud__destroy(other_msgs__msg__SegCloud * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    other_msgs__msg__SegCloud__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
other_msgs__msg__SegCloud__Sequence__init(other_msgs__msg__SegCloud__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  other_msgs__msg__SegCloud * data = NULL;

  if (size) {
    data = (other_msgs__msg__SegCloud *)allocator.zero_allocate(size, sizeof(other_msgs__msg__SegCloud), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = other_msgs__msg__SegCloud__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        other_msgs__msg__SegCloud__fini(&data[i - 1]);
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
other_msgs__msg__SegCloud__Sequence__fini(other_msgs__msg__SegCloud__Sequence * array)
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
      other_msgs__msg__SegCloud__fini(&array->data[i]);
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

other_msgs__msg__SegCloud__Sequence *
other_msgs__msg__SegCloud__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  other_msgs__msg__SegCloud__Sequence * array = (other_msgs__msg__SegCloud__Sequence *)allocator.allocate(sizeof(other_msgs__msg__SegCloud__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = other_msgs__msg__SegCloud__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
other_msgs__msg__SegCloud__Sequence__destroy(other_msgs__msg__SegCloud__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    other_msgs__msg__SegCloud__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
other_msgs__msg__SegCloud__Sequence__are_equal(const other_msgs__msg__SegCloud__Sequence * lhs, const other_msgs__msg__SegCloud__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!other_msgs__msg__SegCloud__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
other_msgs__msg__SegCloud__Sequence__copy(
  const other_msgs__msg__SegCloud__Sequence * input,
  other_msgs__msg__SegCloud__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(other_msgs__msg__SegCloud);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    other_msgs__msg__SegCloud * data =
      (other_msgs__msg__SegCloud *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!other_msgs__msg__SegCloud__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          other_msgs__msg__SegCloud__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!other_msgs__msg__SegCloud__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
