#include "inc/_common.h"

static bool stub = false;
bool hard_stub() {
  return stub;
}

bool hard_lock() {
  stub = true;
  return true;
}

bool hard_unlock() {
  stub = false;
  return true;
}
