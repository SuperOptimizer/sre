#include <stdio.h>

#include "lr35902.h"

int main(int argc, char** argv) {
  lr35902_main();
  printf("hello world\n");
  return 0;
}

#define MINICORO_IMPL
#include "minicoro.h"