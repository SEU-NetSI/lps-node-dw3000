#include "ranging_struct.h"

#include <stdio.h>

#include "task.h"

static set_index_t ranging_table_set_malloc(
    Ranging_Table_Set_t *ranging_table_set) {
  if (ranging_table_set->free_queue_entry == -1) {
    printf("Ranging Table Set is FULL, malloc failed.\r\n");
    return -1;
  } else {
    set_index_t candidate = ranging_table_set->free_queue_entry;
    ranging_table_set->free_queue_entry =
        ranging_table_set->set_data[candidate].next;
    // insert to full queue
    set_index_t temp = ranging_table_set->full_queue_entry;
    ranging_table_set->full_queue_entry = candidate;
    ranging_table_set->set_data[candidate].next = temp;
    return candidate;
  }
}

static bool ranging_table_set_free(Ranging_Table_Set_t *ranging_table_set,
                                   set_index_t item_index) {
  if (-1 == item_index) {
    return true;
  }
  // delete from full queue
  set_index_t pre = ranging_table_set->full_queue_entry;
  if (item_index == pre) {
    ranging_table_set->full_queue_entry = ranging_table_set->set_data[pre].next;
    // insert into empty queue
    ranging_table_set->set_data[item_index].next =
        ranging_table_set->free_queue_entry;
    ranging_table_set->free_queue_entry = item_index;
    ranging_table_set->size = ranging_table_set->size - 1;
    return true;
  } else {
    while (pre != -1) {
      if (ranging_table_set->set_data[pre].next == item_index) {
        ranging_table_set->set_data[pre].next =
            ranging_table_set->set_data[item_index].next;
        // insert into empty queue
        ranging_table_set->set_data[item_index].next =
            ranging_table_set->free_queue_entry;
        ranging_table_set->free_queue_entry = item_index;
        ranging_table_set->size = ranging_table_set->size - 1;
        return true;
      }
      pre = ranging_table_set->set_data[pre].next;
    }
  }
  return false;
}

void ranging_table_init(Ranging_Table_Set_t *ranging_table_set) {
  set_index_t i;
  for (i = 0; i < RANGING_TABLE_SIZE - 1; i++) {
    ranging_table_set->set_data[i].next = i + 1;
  }
  ranging_table_set->set_data[i].next = -1;
  ranging_table_set->free_queue_entry = 0;
  ranging_table_set->full_queue_entry = -1;
  ranging_table_set->size = 0;
}

set_index_t ranging_table_insert(Ranging_Table_Set_t *ranging_table_set,
                                 Ranging_Table_t *table) {
  set_index_t candidate = ranging_table_set_malloc(ranging_table_set);
  if (candidate != -1) {
    memcpy(&ranging_table_set->set_data[candidate].data, table,
           sizeof(Ranging_Table_t));
    ranging_table_set->size++;
  }
  return candidate;
}

set_index_t find_in_ranging_table(Ranging_Table_Set_t *ranging_table_set,
                                  address_t addr) {
  set_index_t iter = ranging_table_set->full_queue_entry;
  while (iter != -1) {
    Ranging_Table_Set_Item_t cur = ranging_table_set->set_data[iter];
    if (cur.data.neighbor_address == addr) {
      break;
    }
    iter = cur.next;
  }
  return iter;
}

bool delete_ranging_tuple_by_index(Ranging_Table_Set_t *ranging_table_set,
                                   set_index_t index) {
  return ranging_table_set_free(ranging_table_set, index);
}

void print_ranging_table_tuple(Ranging_Table_Set_t *tuple) {}

void print_ranging_table(Ranging_Table_Set_t *ranging_table_set) {}

bool ranging_table_clear_expire(Ranging_Table_Set_t *ranging_table_set) {
  set_index_t candidate = ranging_table_set->full_queue_entry;
  Time_t now = xTaskGetTickCount();
  bool has_changed = false;
  while (candidate != -1) {
    Ranging_Table_Set_Item_t temp = ranging_table_set->set_data[candidate];
    if (temp.data.expiration_time < now) {
      set_index_t next_index = temp.next;
      ranging_table_set_free(ranging_table_set, candidate);
      candidate = next_index;
      has_changed = true;
      continue;
    }
    candidate = temp.next;
  }
  return has_changed;
}

void sort_ranging_table(Ranging_Table_Set_t *ranging_table_set) {
  if (ranging_table_set->full_queue_entry == -1) {
    return;
  }
  set_index_t new_head = ranging_table_set->full_queue_entry;
  set_index_t cur = ranging_table_set->set_data[new_head].next;
  ranging_table_set->set_data[new_head].next = -1;
  set_index_t next = -1;
  while (cur != -1) {
    next = ranging_table_set->set_data[cur].next;
    if (ranging_table_set->set_data[cur].data.next_delivery_time <=
        ranging_table_set->set_data[new_head].data.next_delivery_time) {
      ranging_table_set->set_data[cur].next = new_head;
      new_head = cur;
    } else {
      set_index_t start = ranging_table_set->set_data[new_head].next;
      set_index_t pre = new_head;
      while (start != -1 &&
             ranging_table_set->set_data[cur].data.next_delivery_time >
                 ranging_table_set->set_data[start].data.next_delivery_time) {
        pre = start;
        start = ranging_table_set->set_data[start].next;
      }
      ranging_table_set->set_data[cur].next = start;
      ranging_table_set->set_data[pre].next = cur;
    }
    cur = next;
  }
  ranging_table_set->full_queue_entry = new_head;
}