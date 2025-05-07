#pragma once

#include "common.h"


typedef union gpr {
  u128 u128[1];
  s128 s128[1];
  u64 u64[2];
  s64 s64[2];
  u32 u32[4];
  s32 s32[4];
  u16 u16[8];
  s16 s16[8];
  u8 u8[16];
  s8 s8[16];
} gpr;

typedef struct lr35902 {
  union {
    struct {
      union {
        struct {
          union {
            u8 f;
            struct {
              u8 z : 1;
              u8 n : 1;
              u8 h : 1;
              u8 c : 1;
              u8 unused : 4;
            } flags;
          };
          u8 a;
        };
        u16 af;
      };
      union {
        struct {u8 c, b;};
        u16 bc;
      };
      union {
        struct {u8 e, d;};
        u16 de;
      };
      union {
        struct {u8 l, h;};
        u16 hl;
      };
    };
    u8 regs[8];
  };

  u16 pc;
  u16 sp;

  bool ime;
  u8 halt;
  u8 stop;

  u64 cycles;
  u64 remaining_cycles;

  u8* memory;

  u8 (*read8)(struct lr35902* lr35902, u16 addr);
  void (*write8)(struct lr35902* lr35902, u16 addr, u8 data);

  mco_coro* cpu_mco_coro;
  void (*cpu_coro)(mco_coro* co);

} lr35902;

void lr35902_new(lr35902* lr35902);
u8 lr35902_read8(lr35902* lr35902, u16 addr);
void lr35902_write8(lr35902* lr35902, u16 addr, u8 data);
void lr35902_run(lr35902* lr35902, u64 cycles);
void lr35902_destroy(lr35902* lr35902);
void lr35902_cpu_coro(mco_coro* co);
void lr35902_main();