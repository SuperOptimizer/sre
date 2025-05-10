#include "lr35902.h"

void lr35902_new(lr35902* lr35902) {
  memset(lr35902, 0, sizeof(*lr35902));

  mco_desc desc = mco_desc_init(lr35902_cpu_coro, 0);
  desc.user_data = lr35902;

  mco_result res = mco_create(&lr35902->cpu_mco_coro, &desc);
  if (res != MCO_SUCCESS) {
  }
}

void lr35902_run(lr35902* lr35902, u64 cycles) {
  lr35902->remaining_cycles = cycles;

  while (lr35902->remaining_cycles > 0 && mco_status(lr35902->cpu_mco_coro) == MCO_SUSPENDED) {
    mco_result res = mco_resume(lr35902->cpu_mco_coro);
    if (res != MCO_SUCCESS) {
      break;
    }
    printf("remaining cycles %u\n",(u32)lr35902->remaining_cycles);
    lr35902->remaining_cycles--;
    lr35902->cycles++;

    // Synchronize with other components (PPU, timer, audio, etc.)
    // This is where you'd call functions to update other system components
  }
}

void lr35902_destroy(lr35902* cpu) {
  if (cpu->cpu_mco_coro) {
    mco_destroy(cpu->cpu_mco_coro);
    cpu->cpu_mco_coro = NULL;
  }
}

#define FLAG_Z 7  // Zero
#define FLAG_N 6  // Subtract
#define FLAG_H 5  // Half-carry
#define FLAG_C 4  // Carry

#define FLAG_Z_MASK (1 << FLAG_Z)
#define FLAG_N_MASK (1 << FLAG_N)
#define FLAG_H_MASK (1 << FLAG_H)
#define FLAG_C_MASK (1 << FLAG_C)

static void set_flag(lr35902* cpu, u8 flag, bool value) {
    if (value) {
        cpu->f |= (1 << flag);
    } else {
        cpu->f &= ~(1 << flag);
    }
}

static bool get_flag(lr35902* cpu, u8 flag) {
    return (cpu->f & (1 << flag)) != 0;
}

static void update_zero_flag(lr35902* cpu, u8 value) {
    set_flag(cpu, FLAG_Z, value == 0);
}

static void update_add_flags(lr35902* cpu, u8 a, u8 b, bool with_carry) {
    u8 carry = with_carry && get_flag(cpu, FLAG_C) ? 1 : 0;
    u16 result = a + b + carry;

    update_zero_flag(cpu, result & 0xFF);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, ((a & 0x0F) + (b & 0x0F) + carry) > 0x0F);
    set_flag(cpu, FLAG_C, result > 0xFF);
}

static void update_sub_flags(lr35902* cpu, u8 a, u8 b, bool with_carry) {
    u8 carry = with_carry && get_flag(cpu, FLAG_C) ? 1 : 0;
    int result = a - b - carry;

    update_zero_flag(cpu, (result & 0xFF) == 0);
    set_flag(cpu, FLAG_N, true);
    set_flag(cpu, FLAG_H, ((a & 0x0F) - (b & 0x0F) - carry) < 0);
    set_flag(cpu, FLAG_C, result < 0);
}

static u8 inc8(lr35902* cpu, u8 value) {
    u8 result = value + 1;

    update_zero_flag(cpu, result);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, (value & 0x0F) == 0x0F);

    return result;
}

static u8 dec8(lr35902* cpu, u8 value) {
    u8 result = value - 1;

    update_zero_flag(cpu, result);
    set_flag(cpu, FLAG_N, true);
    set_flag(cpu, FLAG_H, (value & 0x0F) == 0);

    return result;
}

static void update_add16_flags(lr35902* cpu, u16 a, u16 b) {
    u32 result = a + b;

    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, ((a & 0xFFF) + (b & 0xFFF)) > 0xFFF);
    set_flag(cpu, FLAG_C, result > 0xFFFF);
}

static u8 rotate_a_special(lr35902* cpu, bool left, bool through_carry) {
    u8 old_carry = get_flag(cpu, FLAG_C) ? 1 : 0;
    u8 value = cpu->a;
    u8 new_carry;
    u8 result;

    if (left) {
        new_carry = (value & 0x80) >> 7;
        if (through_carry) {
            // RLA
            result = (value << 1) | old_carry;
        } else {
            // RLCA
            result = (value << 1) | new_carry;
        }
    } else {
        new_carry = value & 0x01;
        if (through_carry) {
            // RRA
            result = (value >> 1) | (old_carry << 7);
        } else {
            // RRCA
            result = (value >> 1) | (new_carry << 7);
        }
    }

    // Special case: A rotation instructions clear Z, N, H flags
    set_flag(cpu, FLAG_Z, false);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, new_carry);

    return result;
}

static void update_and_flags(lr35902* cpu, u8 result) {
    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, true);  // H is always set for AND
    set_flag(cpu, FLAG_C, false);
}

static void update_xor_flags(lr35902* cpu, u8 result) {
    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, false);
}

static void update_or_flags(lr35902* cpu, u8 result) {
    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, false);
}

static void cp_operation(lr35902* cpu, u8 value) {
    update_sub_flags(cpu, cpu->a, value, false);
}


static u8 do_rlc(lr35902* cpu, u8 value) {
    u8 carry = (value & 0x80) >> 7;
    u8 result = (value << 1) | carry;

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, carry);

    return result;
}

static u8 do_rrc(lr35902* cpu, u8 value) {
    u8 carry = value & 0x01;
    u8 result = (value >> 1) | (carry << 7);

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, carry);

    return result;
}

static u8 do_rl(lr35902* cpu, u8 value) {
    u8 old_carry = get_flag(cpu, FLAG_C) ? 1 : 0;
    u8 new_carry = (value & 0x80) >> 7;
    u8 result = (value << 1) | old_carry;

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, new_carry);

    return result;
}

static u8 do_rr(lr35902* cpu, u8 value) {
    u8 old_carry = get_flag(cpu, FLAG_C) ? 1 : 0;
    u8 new_carry = value & 0x01;
    u8 result = (value >> 1) | (old_carry << 7);

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, new_carry);

    return result;
}

static u8 do_sla(lr35902* cpu, u8 value) {
    u8 carry = (value & 0x80) >> 7;
    u8 result = value << 1;

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, carry);

    return result;
}

static u8 do_sra(lr35902* cpu, u8 value) {
    u8 carry = value & 0x01;
    u8 result = (value >> 1) | (value & 0x80);

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, carry);

    return result;
}

static u8 do_swap(lr35902* cpu, u8 value) {
    u8 result = ((value & 0x0F) << 4) | ((value & 0xF0) >> 4);

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, false);

    return result;
}

static u8 do_srl(lr35902* cpu, u8 value) {
    u8 carry = value & 0x01;
    u8 result = value >> 1;

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, carry);

    return result;
}

static void do_bit(lr35902* cpu, u8 value, u8 bit) {
    u8 result = value & (1 << bit);

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, true);
    // C flag is preserved
}

static u8 do_res(lr35902* cpu, u8 value, u8 bit) {
    return value & ~(1 << bit);
}

static u8 do_set(lr35902* cpu, u8 value, u8 bit) {
    return value | (1 << bit);
}


void handle_cb_prefix(lr35902* cpu, mco_coro* co) {
    u8 cb_opcode = lr35902_read8(cpu, cpu->pc++);
    mco_yield(co);

    u8 op_type = cb_opcode >> 6;  // Top 2 bits define operation type
    u8 bit_num = (cb_opcode >> 3) & 0x07;  // Middle 3 bits define bit number or operation
    u8 reg_num = cb_opcode & 0x07;  // Bottom 3 bits define register

    u8 value = 0;
    bool is_hl = reg_num == 0x06;

    if (is_hl) {
        value = lr35902_read8(cpu, cpu->hl);
        mco_yield(co);
    } else {
        switch (reg_num) {
            case 0: value = cpu->b; break;
            case 1: value = cpu->c; break;
            case 2: value = cpu->d; break;
            case 3: value = cpu->e; break;
            case 4: value = cpu->h; break;
            case 5: value = cpu->l; break;
            case 7: value = cpu->a; break;
            default: break;
        }
    }

    u8 result = 0;

    switch (op_type) {
        case 0:
            switch (bit_num) {
                case 0: result = do_rlc(cpu, value); break;
                case 1: result = do_rrc(cpu, value); break;
                case 2: result = do_rl(cpu, value); break;
                case 3: result = do_rr(cpu, value); break;
                case 4: result = do_sla(cpu, value); break;
                case 5: result = do_sra(cpu, value); break;
                case 6: result = do_swap(cpu, value); break;
                case 7: result = do_srl(cpu, value); break;
            default: break;
            }
            break;

        case 1:
            do_bit(cpu, value, bit_num);
            return; // BIT doesn't store results, so just return

        case 2:
            result = do_res(cpu, value, bit_num);
            break;

        case 3:
            result = do_set(cpu, value, bit_num);
            break;
        default: break;
    }

    if (is_hl) {
        lr35902_write8(cpu, cpu->hl, result);
        mco_yield(co);
    } else {
        switch (reg_num) {
            case 0: cpu->b = result; break;
            case 1: cpu->c = result; break;
            case 2: cpu->d = result; break;
            case 3: cpu->e = result; break;
            case 4: cpu->h = result; break;
            case 5: cpu->l = result; break;
            case 7: cpu->a = result; break;
            default: break;
        }
    }
}

void lr35902_cpu_coro(mco_coro* co) {
  lr35902* lr35902 = mco_get_user_data(co);

  while (1) {
    u8 opcode;
    opcode = lr35902_read8(lr35902, lr35902->pc++);
    mco_yield(co);

    switch (opcode) {
    case 0x00: // NOP - 1 M-cycle
    break;

    case 0x01: // LD BC,d16 - 3 M-cycles
        lr35902->c = lr35902_read8(lr35902, lr35902->pc++);
        mco_yield(co);
        lr35902->b = lr35902_read8(lr35902, lr35902->pc++);
        mco_yield(co);
        break;

    case 0x02: // LD (BC),A - 2 M-cycles
        lr35902_write8(lr35902, lr35902->bc, lr35902->a);
        mco_yield(co);
        break;

    case 0x03: // INC BC - 2 M-cycles
        lr35902->bc++;
        mco_yield(co);
        break;

    case 0x04: // INC B - 1 M-cycle
        lr35902->b = inc8(lr35902, lr35902->b);
        break;

    case 0x05: // DEC B - 1 M-cycle
        lr35902->b = dec8(lr35902, lr35902->b);
        break;

    case 0x06: // LD B,d8 - 2 M-cycles
        lr35902->b = lr35902_read8(lr35902, lr35902->pc++);
        mco_yield(co);
        break;

    case 0x07: // RLCA - 1 M-cycle
        lr35902->a = rotate_a_special(lr35902, true, false);
        break;

    case 0x08: // LD (a16),SP - 5 M-cycles
        {
            u16 addr = lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);

            addr |= (lr35902_read8(lr35902, lr35902->pc++) << 8);
            mco_yield(co);

            lr35902_write8(lr35902, addr, lr35902->sp & 0xFF);
            mco_yield(co);

            lr35902_write8(lr35902, addr + 1, lr35902->sp >> 8);
            mco_yield(co);
        }
        break;

    case 0x09: // ADD HL,BC - 2 M-cycles
        update_add16_flags(lr35902, lr35902->hl, lr35902->bc);
        lr35902->hl += lr35902->bc;
        mco_yield(co);
        break;

    case 0x0A: // LD A,(BC) - 2 M-cycles
        lr35902->a = lr35902_read8(lr35902, lr35902->bc);
        mco_yield(co);
        break;

    case 0x0B: // DEC BC - 2 M-cycles
        lr35902->bc--;
        mco_yield(co);
        break;

    case 0x0C: // INC C - 1 M-cycle
        lr35902->c = inc8(lr35902, lr35902->c);
        break;

    case 0x0D: // DEC C - 1 M-cycle
        lr35902->c = dec8(lr35902, lr35902->c);
        break;

    case 0x0E: // LD C,d8 - 2 M-cycles
        lr35902->c = lr35902_read8(lr35902, lr35902->pc++);
        mco_yield(co);
        break;

    case 0x0F: // RRCA - 1 M-cycle
        lr35902->a = rotate_a_special(lr35902, false, false);
        break;
    case 0x10: break;                              // STOP 0
    case 0x11: // LD DE,d16 - 3 M-cycles
        lr35902->e = lr35902_read8(lr35902, lr35902->pc++);
        mco_yield(co);
        lr35902->d = lr35902_read8(lr35902, lr35902->pc++);
        mco_yield(co);
        break;

    case 0x12: // LD (DE),A - 2 M-cycles
        lr35902_write8(lr35902, lr35902->de, lr35902->a);
        mco_yield(co);
        break;
    case 0x13:  // INC DE - 2 M-cycles
        lr35902->de++;
        mco_yield(co);
        break;
    case 0x14: // INC D - 1 M-cycle
        lr35902->d = inc8(lr35902, lr35902->d);
        break;
    case 0x15:  // INC D - 1 M-cycle
        lr35902->d = dec8(lr35902, lr35902->d);
        break;
    case 0x16: // LD D,d8 - 2 M-cycles
        lr35902->d = lr35902_read8(lr35902, lr35902->pc++);
        mco_yield(co);
        break;
    case 0x17: // RLA - 1 M-cycle
        lr35902->a = rotate_a_special(lr35902, true, true);
        break;

    case 0x18: // JR r8 - 3 M-cycles
    {
        s8 offset = (s8)lr35902_read8(lr35902, lr35902->pc++);
        mco_yield(co);

        lr35902->pc += offset;
        mco_yield(co);
    }
        break;

    case 0x19: // ADD HL,DE - 2 M-cycles
        update_add16_flags(lr35902, lr35902->hl, lr35902->de);
        lr35902->hl += lr35902->de;
        mco_yield(co);
        break;

    case 0x1A: // LD A,(DE) - 2 M-cycles
        lr35902->a = lr35902_read8(lr35902, lr35902->de);
        mco_yield(co);
        break;

    case 0x1B: // DEC DE - 2 M-cycles
        lr35902->de--;
        mco_yield(co);
        break;

    case 0x1C: // INC E - 1 M-cycle
        lr35902->e = inc8(lr35902, lr35902->e);
        break;

    case 0x1D: // DEC E - 1 M-cycle
        lr35902->e = dec8(lr35902, lr35902->e);
        break;

    case 0x1E: // LD E,d8 - 2 M-cycles
        lr35902->e = lr35902_read8(lr35902, lr35902->pc++);
        mco_yield(co);
        break;

    case 0x1F: // RRA - 1 M-cycle
        lr35902->a = rotate_a_special(lr35902, false, true);
        break;
    case 0x20: // JR NZ,r8 - 2/3 M-cycles
    {
        // Fetch the signed offset - 1 M-cycle
        s8 offset = (s8)lr35902_read8(lr35902, lr35902->pc++);
        mco_yield(co);

        // Check condition and branch if needed - 1 M-cycle if not taken, 2 M-cycles if taken
        if (!get_flag(lr35902, FLAG_Z)) {
            // Condition met (Z flag is not set), perform the jump
            lr35902->pc += offset;
            mco_yield(co); // Extra cycle for taken branch
        }
    }
    break;

    case 0x21: // LD HL,d16 - 3 M-cycles
        lr35902->l = lr35902_read8(lr35902, lr35902->pc++);
        mco_yield(co);
        lr35902->h = lr35902_read8(lr35902, lr35902->pc++);
        mco_yield(co);
        break;

    case 0x22: // LD (HL+),A - 2 M-cycles
        lr35902_write8(lr35902, lr35902->hl, lr35902->a);
        lr35902->hl++;
        mco_yield(co);
        break;

    case 0x23: // INC HL - 2 M-cycles
        lr35902->hl++;
        mco_yield(co);
        break;

    case 0x24: // INC H - 1 M-cycle
        lr35902->h = inc8(lr35902, lr35902->h);
        break;

    case 0x25: // DEC H - 1 M-cycle
        lr35902->h = dec8(lr35902, lr35902->h);
        break;

    case 0x26: // LD H,d8 - 2 M-cycles
        lr35902->h = lr35902_read8(lr35902, lr35902->pc++);
        mco_yield(co);
        break;

    case 0x27: // DAA - 1 M-cycle (Decimal Adjust Accumulator)
        {
            u8 adjust = 0;
            bool carry = get_flag(lr35902, FLAG_C);

            if (get_flag(lr35902, FLAG_H) || (!get_flag(lr35902, FLAG_N) && (lr35902->a & 0x0F) > 9)) {
                adjust |= 0x06;
            }

            if (carry || (!get_flag(lr35902, FLAG_N) && lr35902->a > 0x99)) {
                adjust |= 0x60;
                carry = true;
            }

            lr35902->a = get_flag(lr35902, FLAG_N) ? lr35902->a - adjust : lr35902->a + adjust;

            update_zero_flag(lr35902, lr35902->a);
            set_flag(lr35902, FLAG_H, false);
            set_flag(lr35902, FLAG_C, carry);
        }
        break;

    case 0x28: // JR Z,r8 - 2/3 M-cycles
        {
            s8 offset = (s8)lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);

            if (get_flag(lr35902, FLAG_Z)) {
                lr35902->pc += offset;
                mco_yield(co);
            }
        }
        break;

    case 0x29: // ADD HL,HL - 2 M-cycles
        update_add16_flags(lr35902, lr35902->hl, lr35902->hl);
        lr35902->hl += lr35902->hl; // Same as HL *= 2
        mco_yield(co);
        break;

    case 0x2A: // LD A,(HL+) - 2 M-cycles
        lr35902->a = lr35902_read8(lr35902, lr35902->hl);
        lr35902->hl++;
        mco_yield(co);
        break;

    case 0x2B: // DEC HL - 2 M-cycles
        lr35902->hl--;
        mco_yield(co);
        break;

    case 0x2C: // INC L - 1 M-cycle
        lr35902->l = inc8(lr35902, lr35902->l);
        break;

    case 0x2D: // DEC L - 1 M-cycle
        // Already consumed the M-cycle for fetch
        lr35902->l = dec8(lr35902, lr35902->l);
        break;

    case 0x2E: // LD L,d8 - 2 M-cycles
        lr35902->l = lr35902_read8(lr35902, lr35902->pc++);
        mco_yield(co);
        break;

    case 0x2F: // CPL - 1 M-cycle (Complement A)
        lr35902->a = ~lr35902->a;
        set_flag(lr35902, FLAG_N, true);
        set_flag(lr35902, FLAG_H, true);
        break;
    case 0x30: // JR NC,r8 - 2/3 M-cycles
    {
        s8 offset = (s8)lr35902_read8(lr35902, lr35902->pc++);
        mco_yield(co);

        if (!get_flag(lr35902, FLAG_C)) {
            lr35902->pc += offset;
            mco_yield(co);
        }
    }
    break;

    case 0x31: // LD SP,d16 - 3 M-cycles
        {
            u16 value = lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);
            value |= (lr35902_read8(lr35902, lr35902->pc++) << 8);
            lr35902->sp = value;
            mco_yield(co);
        }
        break;

    case 0x32: // LD (HL-),A - 2 M-cycles
        lr35902_write8(lr35902, lr35902->hl, lr35902->a);
        lr35902->hl--;
        mco_yield(co);
        break;

    case 0x33: // INC SP - 2 M-cycles
        lr35902->sp++;
        mco_yield(co);
        break;

    case 0x34: // INC (HL) - 3 M-cycles
        {
            u8 value = lr35902_read8(lr35902, lr35902->hl);
            mco_yield(co);
            value = inc8(lr35902, value);
            lr35902_write8(lr35902, lr35902->hl, value);
            mco_yield(co);
        }
        break;

    case 0x35: // DEC (HL) - 3 M-cycles
        {
            u8 value = lr35902_read8(lr35902, lr35902->hl);
            mco_yield(co);
            value = dec8(lr35902, value);
            lr35902_write8(lr35902, lr35902->hl, value);
            mco_yield(co);
        }
        break;

    case 0x36: // LD (HL),d8 - 3 M-cycles
        {
            u8 value = lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);
            lr35902_write8(lr35902, lr35902->hl, value);
            mco_yield(co);
        }
        break;

    case 0x37: // SCF - 1 M-cycle (Set Carry Flag)
        set_flag(lr35902, FLAG_N, false);
        set_flag(lr35902, FLAG_H, false);
        set_flag(lr35902, FLAG_C, true);
        break;

    case 0x38: // JR C,r8 - 2/3 M-cycles
        {
            s8 offset = (s8)lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);
            if (get_flag(lr35902, FLAG_C)) {
                lr35902->pc += offset;
                mco_yield(co);
            }
        }
        break;

    case 0x39: // ADD HL,SP - 2 M-cycles
        update_add16_flags(lr35902, lr35902->hl, lr35902->sp);
        lr35902->hl += lr35902->sp;
        mco_yield(co);
        break;

    case 0x3A: // LD A,(HL-) - 2 M-cycles
        lr35902->a = lr35902_read8(lr35902, lr35902->hl);
        lr35902->hl--;
        mco_yield(co);
        break;

    case 0x3B: // DEC SP - 2 M-cycles
        lr35902->sp--;
        mco_yield(co);
        break;

    case 0x3C: // INC A - 1 M-cycle
        lr35902->a = inc8(lr35902, lr35902->a);
        break;

    case 0x3D: // DEC A - 1 M-cycle
        lr35902->a = dec8(lr35902, lr35902->a);
        break;

    case 0x3E: // LD A,d8 - 2 M-cycles
        lr35902->a = lr35902_read8(lr35902, lr35902->pc++);
        mco_yield(co);
        break;

    case 0x3F: // CCF - 1 M-cycle (Complement Carry Flag)
        set_flag(lr35902, FLAG_N, false);
        set_flag(lr35902, FLAG_H, false);
        set_flag(lr35902, FLAG_C, !get_flag(lr35902, FLAG_C));
        break;
    case 0x40: // LD B,B - 1 M-cycle
        break;

    case 0x41: // LD B,C - 1 M-cycle
        lr35902->b = lr35902->c;
        break;

    case 0x42: // LD B,D - 1 M-cycle
        lr35902->b = lr35902->d;
        break;

    case 0x43: // LD B,E - 1 M-cycle
        lr35902->b = lr35902->e;
        break;

    case 0x44: // LD B,H - 1 M-cycle
        lr35902->b = lr35902->h;
        break;

    case 0x45: // LD B,L - 1 M-cycle
        lr35902->b = lr35902->l;
        break;

    case 0x46: // LD B,(HL) - 2 M-cycles
        lr35902->b = lr35902_read8(lr35902, lr35902->hl);
        mco_yield(co);
        break;

    case 0x47: // LD B,A - 1 M-cycle
        lr35902->b = lr35902->a;
        break;

    case 0x48: // LD C,B - 1 M-cycle
        lr35902->c = lr35902->b;
        break;

    case 0x49: // LD C,C - 1 M-cycle
        break;

    case 0x4A: // LD C,D - 1 M-cycle
        lr35902->c = lr35902->d;
        break;

    case 0x4B: // LD C,E - 1 M-cycle
        lr35902->c = lr35902->e;
        break;

    case 0x4C: // LD C,H - 1 M-cycle
        lr35902->c = lr35902->h;
        break;

    case 0x4D: // LD C,L - 1 M-cycle
        lr35902->c = lr35902->l;
        break;

    case 0x4E: // LD C,(HL) - 2 M-cycles
        lr35902->c = lr35902_read8(lr35902, lr35902->hl);
        mco_yield(co);
        break;

    case 0x4F: // LD C,A - 1 M-cycle
        lr35902->c = lr35902->a;
        break;

    case 0x50: // LD D,B - 1 M-cycle
        lr35902->d = lr35902->b;
        break;

    case 0x51: // LD D,C - 1 M-cycle
        lr35902->d = lr35902->c;
        break;

    case 0x52: // LD D,D - 1 M-cycle
        break;

    case 0x53: // LD D,E - 1 M-cycle
        lr35902->d = lr35902->e;
        break;

    case 0x54: // LD D,H - 1 M-cycle
        lr35902->d = lr35902->h;
        break;

    case 0x55: // LD D,L - 1 M-cycle
        lr35902->d = lr35902->l;
        break;

    case 0x56: // LD D,(HL) - 2 M-cycles
        lr35902->d = lr35902_read8(lr35902, lr35902->hl);
        mco_yield(co);
        break;

    case 0x57: // LD D,A - 1 M-cycle
        lr35902->d = lr35902->a;
        break;

    case 0x58: // LD E,B - 1 M-cycle
        lr35902->e = lr35902->b;
        break;

    case 0x59: // LD E,C - 1 M-cycle
        lr35902->e = lr35902->c;
        break;

    case 0x5A: // LD E,D - 1 M-cycle
        lr35902->e = lr35902->d;
        break;

    case 0x5B: // LD E,E - 1 M-cycle
        break;

    case 0x5C: // LD E,H - 1 M-cycle
        lr35902->e = lr35902->h;
        break;

    case 0x5D: // LD E,L - 1 M-cycle
        lr35902->e = lr35902->l;
        break;

    case 0x5E: // LD E,(HL) - 2 M-cycles
        lr35902->e = lr35902_read8(lr35902, lr35902->hl);
        mco_yield(co);
        break;

    case 0x5F: // LD E,A - 1 M-cycle
        lr35902->e = lr35902->a;
        break;

    case 0x60: // LD H,B - 1 M-cycle
        lr35902->h = lr35902->b;
        break;

    case 0x61: // LD H,C - 1 M-cycle
        lr35902->h = lr35902->c;
        break;

    case 0x62: // LD H,D - 1 M-cycle
        lr35902->h = lr35902->d;
        break;

    case 0x63: // LD H,E - 1 M-cycle
        lr35902->h = lr35902->e;
        break;

    case 0x64: // LD H,H - 1 M-cycle
        break;

    case 0x65: // LD H,L - 1 M-cycle
        lr35902->h = lr35902->l;
        break;

    case 0x66: // LD H,(HL) - 2 M-cycles
        lr35902->h = lr35902_read8(lr35902, lr35902->hl);
        mco_yield(co);
        break;

    case 0x67: // LD H,A - 1 M-cycle
        lr35902->h = lr35902->a;
        break;

    case 0x68: // LD L,B - 1 M-cycle
        lr35902->l = lr35902->b;
        break;

    case 0x69: // LD L,C - 1 M-cycle
        lr35902->l = lr35902->c;
        break;

    case 0x6A: // LD L,D - 1 M-cycle
        lr35902->l = lr35902->d;
        break;

    case 0x6B: // LD L,E - 1 M-cycle
        lr35902->l = lr35902->e;
        break;

    case 0x6C: // LD L,H - 1 M-cycle
        lr35902->l = lr35902->h;
        break;

    case 0x6D: // LD L,L - 1 M-cycle
        break;

    case 0x6E: // LD L,(HL) - 2 M-cycles
        lr35902->l = lr35902_read8(lr35902, lr35902->hl);
        mco_yield(co);
        break;

    case 0x6F: // LD L,A - 1 M-cycle
        lr35902->l = lr35902->a;
        break;

    case 0x70: // LD (HL),B - 2 M-cycles
        lr35902_write8(lr35902, lr35902->hl, lr35902->b);
        mco_yield(co);
        break;

    case 0x71: // LD (HL),C - 2 M-cycles
        lr35902_write8(lr35902, lr35902->hl, lr35902->c);
        mco_yield(co);
        break;

    case 0x72: // LD (HL),D - 2 M-cycles
        lr35902_write8(lr35902, lr35902->hl, lr35902->d);
        mco_yield(co);
        break;

    case 0x73: // LD (HL),E - 2 M-cycles
        lr35902_write8(lr35902, lr35902->hl, lr35902->e);
        mco_yield(co);
        break;

    case 0x74: // LD (HL),H - 2 M-cycles
        lr35902_write8(lr35902, lr35902->hl, lr35902->h);
        mco_yield(co);
        break;

    case 0x75: // LD (HL),L - 2 M-cycles
        lr35902_write8(lr35902, lr35902->hl, lr35902->l);
        mco_yield(co);
        break;

    case 0x76: // HALT - 1 M-cycle
        // Already consumed the M-cycle for fetch
        // Set the HALT state which stops the CPU until an interrupt occurs
        //lr35902->halt = 1;
        // Note: In a real implementation, we would break out of the emulation loop here
        // and wait for an interrupt, but for our purposes we'll just set the flag
        break;

    case 0x77: // LD (HL),A - 2 M-cycles
        lr35902_write8(lr35902, lr35902->hl, lr35902->a);
        mco_yield(co);
        break;

    case 0x78: // LD A,B - 1 M-cycle
        lr35902->a = lr35902->b;
        break;

    case 0x79: // LD A,C - 1 M-cycle
        lr35902->a = lr35902->c;
        break;

    case 0x7A: // LD A,D - 1 M-cycle
        lr35902->a = lr35902->d;
        break;

    case 0x7B: // LD A,E - 1 M-cycle
        lr35902->a = lr35902->e;
        break;

    case 0x7C: // LD A,H - 1 M-cycle
        lr35902->a = lr35902->h;
        break;

    case 0x7D: // LD A,L - 1 M-cycle
        lr35902->a = lr35902->l;
        break;

    case 0x7E: // LD A,(HL) - 2 M-cycles
        lr35902->a = lr35902_read8(lr35902, lr35902->hl);
        mco_yield(co);
        break;

    case 0x7F: // LD A,A - 1 M-cycle
        break;
    case 0x80: // ADD A,B - 1 M-cycle
        {
            u8 value = lr35902->b;
            update_add_flags(lr35902, lr35902->a, value, false);
            lr35902->a += value;
        }
        break;

    case 0x81: // ADD A,C - 1 M-cycle
        {
            u8 value = lr35902->c;
            update_add_flags(lr35902, lr35902->a, value, false);
            lr35902->a += value;
        }
        break;

    case 0x82: // ADD A,D - 1 M-cycle
        {
            u8 value = lr35902->d;
            update_add_flags(lr35902, lr35902->a, value, false);
            lr35902->a += value;
        }
        break;

    case 0x83: // ADD A,E - 1 M-cycle
        {
            u8 value = lr35902->e;
            update_add_flags(lr35902, lr35902->a, value, false);
            lr35902->a += value;
        }
        break;

    case 0x84: // ADD A,H - 1 M-cycle
        {
            u8 value = lr35902->h;
            update_add_flags(lr35902, lr35902->a, value, false);
            lr35902->a += value;
        }
        break;

    case 0x85: // ADD A,L - 1 M-cycle
        {
            u8 value = lr35902->l;
            update_add_flags(lr35902, lr35902->a, value, false);
            lr35902->a += value;
        }
        break;

    case 0x86: // ADD A,(HL) - 2 M-cycles
        {
            u8 value = lr35902_read8(lr35902, lr35902->hl);
            update_add_flags(lr35902, lr35902->a, value, false);
            lr35902->a += value;
            mco_yield(co);
        }
        break;

    case 0x87: // ADD A,A - 1 M-cycle
        {
            u8 value = lr35902->a;
            update_add_flags(lr35902, lr35902->a, value, false);
            lr35902->a += value; // Same as A *= 2
        }
        break;

    case 0x88: // ADC A,B - 1 M-cycle
        {
            u8 value = lr35902->b;
            update_add_flags(lr35902, lr35902->a, value, true);
            lr35902->a += value + (get_flag(lr35902, FLAG_C) ? 1 : 0);
        }
        break;

    case 0x89: // ADC A,C - 1 M-cycle
        {
            u8 value = lr35902->c;
            update_add_flags(lr35902, lr35902->a, value, true);
            lr35902->a += value + (get_flag(lr35902, FLAG_C) ? 1 : 0);
        }
        break;

    case 0x8A: // ADC A,D - 1 M-cycle
        {
            u8 value = lr35902->d;
            update_add_flags(lr35902, lr35902->a, value, true);
            lr35902->a += value + (get_flag(lr35902, FLAG_C) ? 1 : 0);
        }
        break;

    case 0x8B: // ADC A,E - 1 M-cycle
        {
            u8 value = lr35902->e;
            update_add_flags(lr35902, lr35902->a, value, true);
            lr35902->a += value + (get_flag(lr35902, FLAG_C) ? 1 : 0);
        }
        break;

    case 0x8C: // ADC A,H - 1 M-cycle
        {
            u8 value = lr35902->h;
            update_add_flags(lr35902, lr35902->a, value, true);
            lr35902->a += value + (get_flag(lr35902, FLAG_C) ? 1 : 0);
        }
        break;

    case 0x8D: // ADC A,L - 1 M-cycle
        {
            u8 value = lr35902->l;
            update_add_flags(lr35902, lr35902->a, value, true);
            lr35902->a += value + (get_flag(lr35902, FLAG_C) ? 1 : 0);
        }
        break;

    case 0x8E: // ADC A,(HL) - 2 M-cycles
        {
            u8 value = lr35902_read8(lr35902, lr35902->hl);
            update_add_flags(lr35902, lr35902->a, value, true);
            lr35902->a += value + (get_flag(lr35902, FLAG_C) ? 1 : 0);
            mco_yield(co);
        }
        break;

    case 0x8F: // ADC A,A - 1 M-cycle
        {
            u8 value = lr35902->a;
            update_add_flags(lr35902, lr35902->a, value, true);
            lr35902->a += value + (get_flag(lr35902, FLAG_C) ? 1 : 0);
        }
        break;
    case 0x90: // SUB B - 1 M-cycle
        {
            u8 value = lr35902->b;
            update_sub_flags(lr35902, lr35902->a, value, false);
            lr35902->a -= value;
        }
        break;

    case 0x91: // SUB C - 1 M-cycle
        {
            u8 value = lr35902->c;
            update_sub_flags(lr35902, lr35902->a, value, false);
            lr35902->a -= value;
        }
        break;

    case 0x92: // SUB D - 1 M-cycle
        {
            u8 value = lr35902->d;
            update_sub_flags(lr35902, lr35902->a, value, false);
            lr35902->a -= value;
        }
        break;

    case 0x93: // SUB E - 1 M-cycle
        {
            u8 value = lr35902->e;
            update_sub_flags(lr35902, lr35902->a, value, false);
            lr35902->a -= value;
        }
        break;

    case 0x94: // SUB H - 1 M-cycle
        {
            u8 value = lr35902->h;
            update_sub_flags(lr35902, lr35902->a, value, false);
            lr35902->a -= value;
        }
        break;

    case 0x95: // SUB L - 1 M-cycle
        {
            u8 value = lr35902->l;
            update_sub_flags(lr35902, lr35902->a, value, false);
            lr35902->a -= value;
        }
        break;

    case 0x96: // SUB (HL) - 2 M-cycles
        {
            u8 value = lr35902_read8(lr35902, lr35902->hl);
            update_sub_flags(lr35902, lr35902->a, value, false);
            lr35902->a -= value;
            mco_yield(co);
        }
        break;

    case 0x97: // SUB A - 1 M-cycle
        {
            update_sub_flags(lr35902, lr35902->a, lr35902->a, false);
            lr35902->a = 0; // A - A = 0
        }
        break;

    case 0x98: // SBC A,B - 1 M-cycle
        {
            u8 value = lr35902->b;
            u8 carry = get_flag(lr35902, FLAG_C) ? 1 : 0;
            update_sub_flags(lr35902, lr35902->a, value, true);
            lr35902->a -= (value + carry);
        }
        break;

    case 0x99: // SBC A,C - 1 M-cycle
        {
            u8 value = lr35902->c;
            u8 carry = get_flag(lr35902, FLAG_C) ? 1 : 0;
            update_sub_flags(lr35902, lr35902->a, value, true);
            lr35902->a -= (value + carry);
        }
        break;

    case 0x9A: // SBC A,D - 1 M-cycle
        {
            u8 value = lr35902->d;
            u8 carry = get_flag(lr35902, FLAG_C) ? 1 : 0;
            update_sub_flags(lr35902, lr35902->a, value, true);
            lr35902->a -= (value + carry);
        }
        break;

    case 0x9B: // SBC A,E - 1 M-cycle
        {
            u8 value = lr35902->e;
            u8 carry = get_flag(lr35902, FLAG_C) ? 1 : 0;
            update_sub_flags(lr35902, lr35902->a, value, true);
            lr35902->a -= (value + carry);
        }
        break;

    case 0x9C: // SBC A,H - 1 M-cycle
        {
            u8 value = lr35902->h;
            u8 carry = get_flag(lr35902, FLAG_C) ? 1 : 0;
            update_sub_flags(lr35902, lr35902->a, value, true);
            lr35902->a -= (value + carry);
        }
        break;

    case 0x9D: // SBC A,L - 1 M-cycle
        {
            u8 value = lr35902->l;
            u8 carry = get_flag(lr35902, FLAG_C) ? 1 : 0;
            update_sub_flags(lr35902, lr35902->a, value, true);
            lr35902->a -= (value + carry);
        }
        break;

    case 0x9E: // SBC A,(HL) - 2 M-cycles
        {
            u8 value = lr35902_read8(lr35902, lr35902->hl);
            u8 carry = get_flag(lr35902, FLAG_C) ? 1 : 0;
            update_sub_flags(lr35902, lr35902->a, value, true);
            lr35902->a -= (value + carry);
            mco_yield(co);
        }
        break;

    case 0x9F: // SBC A,A - 1 M-cycle
        {
            u8 value = lr35902->a;
            u8 carry = get_flag(lr35902, FLAG_C) ? 1 : 0;
            update_sub_flags(lr35902, lr35902->a, value, true);
            lr35902->a = -carry; // A - A - carry = 0 - carry
        }
        break;

    case 0xA0: // AND B - 1 M-cycle
        lr35902->a &= lr35902->b;
        update_and_flags(lr35902, lr35902->a);
        break;

    case 0xA1: // AND C - 1 M-cycle
        lr35902->a &= lr35902->c;
        update_and_flags(lr35902, lr35902->a);
        break;

    case 0xA2: // AND D - 1 M-cycle
        lr35902->a &= lr35902->d;
        update_and_flags(lr35902, lr35902->a);
        break;

    case 0xA3: // AND E - 1 M-cycle
        lr35902->a &= lr35902->e;
        update_and_flags(lr35902, lr35902->a);
        break;

    case 0xA4: // AND H - 1 M-cycle
        lr35902->a &= lr35902->h;
        update_and_flags(lr35902, lr35902->a);
        break;

    case 0xA5: // AND L - 1 M-cycle
        lr35902->a &= lr35902->l;
        update_and_flags(lr35902, lr35902->a);
        break;

    case 0xA6: // AND (HL) - 2 M-cycles
        {
            u8 value = lr35902_read8(lr35902, lr35902->hl);
            lr35902->a &= value;
            update_and_flags(lr35902, lr35902->a);
            mco_yield(co);
        }
        break;

    case 0xA7: // AND A - 1 M-cycle
        update_and_flags(lr35902, lr35902->a);
        break;

    case 0xA8: // XOR B - 1 M-cycle
        lr35902->a ^= lr35902->b;
        update_xor_flags(lr35902, lr35902->a);
        break;

    case 0xA9: // XOR C - 1 M-cycle
        lr35902->a ^= lr35902->c;
        update_xor_flags(lr35902, lr35902->a);
        break;

    case 0xAA: // XOR D - 1 M-cycle
        lr35902->a ^= lr35902->d;
        update_xor_flags(lr35902, lr35902->a);
        break;

    case 0xAB: // XOR E - 1 M-cycle
        lr35902->a ^= lr35902->e;
        update_xor_flags(lr35902, lr35902->a);
        break;

    case 0xAC: // XOR H - 1 M-cycle
        lr35902->a ^= lr35902->h;
        update_xor_flags(lr35902, lr35902->a);
        break;

    case 0xAD: // XOR L - 1 M-cycle
        lr35902->a ^= lr35902->l;
        update_xor_flags(lr35902, lr35902->a);
        break;

    case 0xAE: // XOR (HL) - 2 M-cycles
        {
            u8 value = lr35902_read8(lr35902, lr35902->hl);
            lr35902->a ^= value;
            update_xor_flags(lr35902, lr35902->a);
            mco_yield(co);
        }
        break;

    case 0xAF: // XOR A - 1 M-cycle
        lr35902->a = 0;
        update_xor_flags(lr35902, lr35902->a);
        break;

    case 0xB0: // OR B - 1 M-cycle
        lr35902->a |= lr35902->b;
        update_or_flags(lr35902, lr35902->a);
        break;

    case 0xB1: // OR C - 1 M-cycle
        lr35902->a |= lr35902->c;
        update_or_flags(lr35902, lr35902->a);
        break;

    case 0xB2: // OR D - 1 M-cycle
        lr35902->a |= lr35902->d;
        update_or_flags(lr35902, lr35902->a);
        break;

    case 0xB3: // OR E - 1 M-cycle
        lr35902->a |= lr35902->e;
        update_or_flags(lr35902, lr35902->a);
        break;

    case 0xB4: // OR H - 1 M-cycle
        lr35902->a |= lr35902->h;
        update_or_flags(lr35902, lr35902->a);
        break;

    case 0xB5: // OR L - 1 M-cycle
        lr35902->a |= lr35902->l;
        update_or_flags(lr35902, lr35902->a);
        break;

    case 0xB6: // OR (HL) - 2 M-cycles
        {
            u8 value = lr35902_read8(lr35902, lr35902->hl);
            lr35902->a |= value;
            update_or_flags(lr35902, lr35902->a);
            mco_yield(co);
        }
        break;

    case 0xB7: // OR A - 1 M-cycle
        update_or_flags(lr35902, lr35902->a);
        break;

    case 0xB8: // CP B - 1 M-cycle
        cp_operation(lr35902, lr35902->b);
        break;

    case 0xB9: // CP C - 1 M-cycle
        cp_operation(lr35902, lr35902->c);
        break;

    case 0xBA: // CP D - 1 M-cycle
        cp_operation(lr35902, lr35902->d);
        break;

    case 0xBB: // CP E - 1 M-cycle
        cp_operation(lr35902, lr35902->e);
        break;

    case 0xBC: // CP H - 1 M-cycle
        cp_operation(lr35902, lr35902->h);
        break;

    case 0xBD: // CP L - 1 M-cycle
        cp_operation(lr35902, lr35902->l);
        break;

    case 0xBE: // CP (HL) - 2 M-cycles
        {
            u8 value = lr35902_read8(lr35902, lr35902->hl);
            cp_operation(lr35902, value);
            mco_yield(co);
        }
        break;

    case 0xBF: // CP A - 1 M-cycle
        set_flag(lr35902, FLAG_Z, true);
        set_flag(lr35902, FLAG_N, true);
        set_flag(lr35902, FLAG_H, false);
        set_flag(lr35902, FLAG_C, false);
        break;
    case 0xC0: // RET NZ - 5/2 M-cycles
        mco_yield(co);

        if (!get_flag(lr35902, FLAG_Z)) {
            u16 addr = lr35902_read8(lr35902, lr35902->sp++);
            mco_yield(co);

            addr |= (lr35902_read8(lr35902, lr35902->sp++) << 8);
            mco_yield(co);

            lr35902->pc = addr;
            mco_yield(co);
        }
        break;

    case 0xC1: // POP BC - 3 M-cycles
        lr35902->c = lr35902_read8(lr35902, lr35902->sp++);
        mco_yield(co);

        lr35902->b = lr35902_read8(lr35902, lr35902->sp++);
        mco_yield(co);
        break;

    case 0xC2: // JP NZ,a16 - 4/3 M-cycles
        {
            u16 addr = lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);

            addr |= (lr35902_read8(lr35902, lr35902->pc++) << 8);
            mco_yield(co);

            if (!get_flag(lr35902, FLAG_Z)) {
                lr35902->pc = addr;
                mco_yield(co);
            }
        }
        break;

    case 0xC3: // JP a16 - 4 M-cycles
        {
            u16 addr = lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);

            addr |= (lr35902_read8(lr35902, lr35902->pc++) << 8);
            mco_yield(co);

            lr35902->pc = addr;
            mco_yield(co);
        }
        break;

    case 0xC4: // CALL NZ,a16 - 6/3 M-cycles
        {
            u16 addr = lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);

            addr |= (lr35902_read8(lr35902, lr35902->pc++) << 8);
            mco_yield(co);

            if (!get_flag(lr35902, FLAG_Z)) {
                mco_yield(co);

                lr35902->sp--;
                lr35902_write8(lr35902, lr35902->sp, lr35902->pc >> 8);
                mco_yield(co);

                lr35902->sp--;
                lr35902_write8(lr35902, lr35902->sp, lr35902->pc & 0xFF);
                mco_yield(co);

                lr35902->pc = addr;
            }
        }
        break;

    case 0xC5: // PUSH BC - 4 M-cycles
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->b);
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->c);
        mco_yield(co);
        break;

    case 0xC6: // ADD A,d8 - 2 M-cycles
        {
            u8 value = lr35902_read8(lr35902, lr35902->pc++);
            update_add_flags(lr35902, lr35902->a, value, false);
            lr35902->a += value;
            mco_yield(co);
        }
        break;

    case 0xC7: // RST 00H - 4 M-cycles (call to 0x0000)
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->pc >> 8);
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->pc & 0xFF);
        mco_yield(co);

        lr35902->pc = 0x0000;
        break;

    case 0xC8: // RET Z - 5/2 M-cycles
        mco_yield(co);

        if (get_flag(lr35902, FLAG_Z)) {
            u16 addr = lr35902_read8(lr35902, lr35902->sp++);
            mco_yield(co);

            addr |= (lr35902_read8(lr35902, lr35902->sp++) << 8);
            mco_yield(co);

            lr35902->pc = addr;
            mco_yield(co);
        }
        break;

    case 0xC9: // RET - 4 M-cycles
        mco_yield(co);

        {
            u16 addr = lr35902_read8(lr35902, lr35902->sp++);
            mco_yield(co);

            addr |= (lr35902_read8(lr35902, lr35902->sp++) << 8);
            mco_yield(co);

            lr35902->pc = addr;
        }
        break;

    case 0xCA: // JP Z,a16 - 4/3 M-cycles
        {
            u16 addr = lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);

            addr |= (lr35902_read8(lr35902, lr35902->pc++) << 8);
            mco_yield(co);

            if (get_flag(lr35902, FLAG_Z)) {
                lr35902->pc = addr;
                mco_yield(co);
            }
        }
        break;

    case 0xCB: // PREFIX CB - 1+X M-cycles
        handle_cb_prefix(lr35902, co);
        break;

    case 0xCC: // CALL Z,a16 - 6/3 M-cycles
        {
            u16 addr = lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);

            addr |= (lr35902_read8(lr35902, lr35902->pc++) << 8);
            mco_yield(co);

            if (get_flag(lr35902, FLAG_Z)) {
                mco_yield(co);

                lr35902->sp--;
                lr35902_write8(lr35902, lr35902->sp, lr35902->pc >> 8);
                mco_yield(co);

                lr35902->sp--;
                lr35902_write8(lr35902, lr35902->sp, lr35902->pc & 0xFF);
                mco_yield(co);

                lr35902->pc = addr;
            }
        }
        break;

    case 0xCD: // CALL a16 - 6 M-cycles
        {
            u16 addr = lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);
            addr |= (lr35902_read8(lr35902, lr35902->pc++) << 8);
            mco_yield(co);

            mco_yield(co);

            lr35902->sp--;
            lr35902_write8(lr35902, lr35902->sp, lr35902->pc >> 8);
            mco_yield(co);

            lr35902->sp--;
            lr35902_write8(lr35902, lr35902->sp, lr35902->pc & 0xFF);
            mco_yield(co);

            lr35902->pc = addr;
        }
        break;

    case 0xCE: // ADC A,d8 - 2 M-cycles
        {
            u8 value = lr35902_read8(lr35902, lr35902->pc++);
            update_add_flags(lr35902, lr35902->a, value, true);
            lr35902->a += value + (get_flag(lr35902, FLAG_C) ? 1 : 0);
            mco_yield(co);
        }
        break;

    case 0xCF: // RST 08H - 4 M-cycles (call to 0x0008)
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->pc >> 8);
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->pc & 0xFF);
        mco_yield(co);

        lr35902->pc = 0x0008;
        break;
    case 0xD0: // RET NC - 5/2 M-cycles
        mco_yield(co);

        if (!get_flag(lr35902, FLAG_C)) {
            u16 addr = lr35902_read8(lr35902, lr35902->sp++);
            mco_yield(co);

            addr |= (lr35902_read8(lr35902, lr35902->sp++) << 8);
            mco_yield(co);

            lr35902->pc = addr;
            mco_yield(co);
        }
        break;

    case 0xD1: // POP DE - 3 M-cycles
        lr35902->e = lr35902_read8(lr35902, lr35902->sp++);
        mco_yield(co);

        lr35902->d = lr35902_read8(lr35902, lr35902->sp++);
        mco_yield(co);
        break;

    case 0xD2: // JP NC,a16 - 4/3 M-cycles
        {
            u16 addr = lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);

            addr |= (lr35902_read8(lr35902, lr35902->pc++) << 8);
            mco_yield(co);

            if (!get_flag(lr35902, FLAG_C)) {
                lr35902->pc = addr;
                mco_yield(co);
            }
        }
        break;

    case 0xD3: // (Illegal) - 1 M-cycle
        break;

    case 0xD4: // CALL NC,a16 - 6/3 M-cycles
        {
            u16 addr = lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);

            addr |= (lr35902_read8(lr35902, lr35902->pc++) << 8);
            mco_yield(co);

            if (!get_flag(lr35902, FLAG_C)) {
                mco_yield(co);

                lr35902->sp--;
                lr35902_write8(lr35902, lr35902->sp, lr35902->pc >> 8);
                mco_yield(co);

                lr35902->sp--;
                lr35902_write8(lr35902, lr35902->sp, lr35902->pc & 0xFF);
                mco_yield(co);

                lr35902->pc = addr;
            }
        }
        break;

    case 0xD5: // PUSH DE - 4 M-cycles
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->d);
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->e);
        mco_yield(co);
        break;

    case 0xD6: // SUB d8 - 2 M-cycles
        {
            u8 value = lr35902_read8(lr35902, lr35902->pc++);
            update_sub_flags(lr35902, lr35902->a, value, false);
            lr35902->a -= value;
            mco_yield(co);
        }
        break;

    case 0xD7: // RST 10H - 4 M-cycles (call to 0x0010)
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->pc >> 8);
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->pc & 0xFF);
        mco_yield(co);

        lr35902->pc = 0x0010;
        break;

    case 0xD8: // RET C - 5/2 M-cycles
        mco_yield(co);

        if (get_flag(lr35902, FLAG_C)) {
            u16 addr = lr35902_read8(lr35902, lr35902->sp++);
            mco_yield(co);

            addr |= (lr35902_read8(lr35902, lr35902->sp++) << 8);
            mco_yield(co);

            lr35902->pc = addr;
            mco_yield(co);
        }
        break;

    case 0xD9: // RETI - 4 M-cycles (Return from Interrupt)
        mco_yield(co);

        {
            u16 addr = lr35902_read8(lr35902, lr35902->sp++);
            mco_yield(co);

            addr |= (lr35902_read8(lr35902, lr35902->sp++) << 8);
            mco_yield(co);

            lr35902->pc = addr;
            lr35902->ime = true;
        }
        break;

    case 0xDA: // JP C,a16 - 4/3 M-cycles
        {
            u16 addr = lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);

            addr |= (lr35902_read8(lr35902, lr35902->pc++) << 8);
            mco_yield(co);

            if (get_flag(lr35902, FLAG_C)) {
                lr35902->pc = addr;
                mco_yield(co);
            }
        }
        break;

    case 0xDB: // (Illegal) - 1 M-cycle
        break;

    case 0xDC: // CALL C,a16 - 6/3 M-cycles
        {
            u16 addr = lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);

            addr |= (lr35902_read8(lr35902, lr35902->pc++) << 8);
            mco_yield(co);

            if (get_flag(lr35902, FLAG_C)) {
                mco_yield(co);

                lr35902->sp--;
                lr35902_write8(lr35902, lr35902->sp, lr35902->pc >> 8);
                mco_yield(co);

                lr35902->sp--;
                lr35902_write8(lr35902, lr35902->sp, lr35902->pc & 0xFF);
                mco_yield(co);

                lr35902->pc = addr;
            }
        }
        break;

    case 0xDD: // (Illegal) - 1 M-cycle
        break;

    case 0xDE: // SBC A,d8 - 2 M-cycles
        {
            u8 value = lr35902_read8(lr35902, lr35902->pc++);
            u8 carry = get_flag(lr35902, FLAG_C) ? 1 : 0;
            update_sub_flags(lr35902, lr35902->a, value, true);
            lr35902->a -= (value + carry);
            mco_yield(co);
        }
        break;

    case 0xDF: // RST 18H - 4 M-cycles (call to 0x0018)
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->pc >> 8);
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->pc & 0xFF);
        mco_yield(co);

        lr35902->pc = 0x0018;
        break;
    case 0xE0: // LDH (a8),A - 3 M-cycles (Store A to address 0xFF00+a8)
        {
            u8 offset = lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);

            lr35902_write8(lr35902, 0xFF00 + offset, lr35902->a);
            mco_yield(co);
        }
        break;

    case 0xE1: // POP HL - 3 M-cycles
        lr35902->l = lr35902_read8(lr35902, lr35902->sp++);
        mco_yield(co);

        lr35902->h = lr35902_read8(lr35902, lr35902->sp++);
        mco_yield(co);
        break;

    case 0xE2: // LD (C),A - 2 M-cycles (Store A to address 0xFF00+C)
        lr35902_write8(lr35902, 0xFF00 + lr35902->c, lr35902->a);
        mco_yield(co);
        break;

    case 0xE3: // (Illegal) - 1 M-cycle
        break;

    case 0xE4: // (Illegal) - 1 M-cycle
        break;

    case 0xE5: // PUSH HL - 4 M-cycles
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->h);
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->l);
        mco_yield(co);
        break;

    case 0xE6: // AND d8 - 2 M-cycles
        {
            u8 value = lr35902_read8(lr35902, lr35902->pc++);
            lr35902->a &= value;
            update_and_flags(lr35902, lr35902->a);
            mco_yield(co);
        }
        break;

    case 0xE7: // RST 20H - 4 M-cycles (call to 0x0020)
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->pc >> 8);
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->pc & 0xFF);
        mco_yield(co);

        lr35902->pc = 0x0020;
        break;

    case 0xE8: // ADD SP,r8 - 4 M-cycles
        {
            s8 offset = (s8)lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);

            set_flag(lr35902, FLAG_Z, false);
            set_flag(lr35902, FLAG_N, false);
            set_flag(lr35902, FLAG_H, (lr35902->sp & 0xF) + (offset & 0xF) > 0xF);
            set_flag(lr35902, FLAG_C, (lr35902->sp & 0xFF) + (offset & 0xFF) > 0xFF);

            lr35902->sp += offset;
            mco_yield(co);
            mco_yield(co);
        }
        break;

    case 0xE9: // JP (HL) - 1 M-cycle
        lr35902->pc = lr35902->hl;
        break;

    case 0xEA: // LD (a16),A - 4 M-cycles
        {
            u16 addr = lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);

            addr |= (lr35902_read8(lr35902, lr35902->pc++) << 8);
            mco_yield(co);

            lr35902_write8(lr35902, addr, lr35902->a);
            mco_yield(co);
        }
        break;

    case 0xEB: // (Illegal) - 1 M-cycle
        break;

    case 0xEC: // (Illegal) - 1 M-cycle
        break;

    case 0xED: // (Illegal) - 1 M-cycle
        break;

    case 0xEE: // XOR d8 - 2 M-cycles
        {
            u8 value = lr35902_read8(lr35902, lr35902->pc++);
            lr35902->a ^= value;
            update_xor_flags(lr35902, lr35902->a);
            mco_yield(co);
        }
        break;

    case 0xEF: // RST 28H - 4 M-cycles (call to 0x0028)
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->pc >> 8);
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->pc & 0xFF);
        mco_yield(co);

        lr35902->pc = 0x0028;
        break;
    case 0xF0: // LDH A,(a8) - 3 M-cycles (Load A from address 0xFF00+a8)
        {
            u8 offset = lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);

            lr35902->a = lr35902_read8(lr35902, 0xFF00 + offset);
            mco_yield(co);
        }
        break;

    case 0xF1: // POP AF - 3 M-cycles
        lr35902->f = lr35902_read8(lr35902, lr35902->sp++) & 0xF0; // Lower 4 bits of F are always 0
        mco_yield(co);

        lr35902->a = lr35902_read8(lr35902, lr35902->sp++);
        mco_yield(co);
        break;

    case 0xF2: // LD A,(C) - 2 M-cycles (Load A from address 0xFF00+C)
        lr35902->a = lr35902_read8(lr35902, 0xFF00 + lr35902->c);
        mco_yield(co);
        break;

    case 0xF3: // DI - 1 M-cycle (Disable Interrupts)
        lr35902->ime = false;
        break;

    case 0xF4: // (Illegal) - 1 M-cycle
        break;

    case 0xF5: // PUSH AF - 4 M-cycles
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->a);
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->f);
        mco_yield(co);
        break;

    case 0xF6: // OR d8 - 2 M-cycles
        {
            u8 value = lr35902_read8(lr35902, lr35902->pc++);
            lr35902->a |= value;
            update_or_flags(lr35902, lr35902->a);
            mco_yield(co);
        }
        break;

    case 0xF7: // RST 30H - 4 M-cycles (call to 0x0030)
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->pc >> 8);
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->pc & 0xFF);
        mco_yield(co);

        lr35902->pc = 0x0030;
        break;

    case 0xF8: // LD HL,SP+r8 - 3 M-cycles
        {
            s8 offset = (s8)lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);

            set_flag(lr35902, FLAG_Z, false);
            set_flag(lr35902, FLAG_N, false);
            set_flag(lr35902, FLAG_H, (lr35902->sp & 0xF) + (offset & 0xF) > 0xF);
            set_flag(lr35902, FLAG_C, (lr35902->sp & 0xFF) + (offset & 0xFF) > 0xFF);

            lr35902->hl = lr35902->sp + offset;
            mco_yield(co);
        }
        break;

    case 0xF9: // LD SP,HL - 2 M-cycles
        lr35902->sp = lr35902->hl;
        mco_yield(co);
        break;

    case 0xFA: // LD A,(a16) - 4 M-cycles
        {
            u16 addr = lr35902_read8(lr35902, lr35902->pc++);
            mco_yield(co);

            addr |= (lr35902_read8(lr35902, lr35902->pc++) << 8);
            mco_yield(co);

            lr35902->a = lr35902_read8(lr35902, addr);
            mco_yield(co);
        }
        break;

    case 0xFB: // EI - 1 M-cycle (Enable Interrupts)
        lr35902->ime = true;
        break;

    case 0xFC: // (Illegal) - 1 M-cycle
        break;

    case 0xFD: // (Illegal) - 1 M-cycle
        break;

    case 0xFE: // CP d8 - 2 M-cycles
        {
            u8 value = lr35902_read8(lr35902, lr35902->pc++);
            cp_operation(lr35902, value);
            mco_yield(co);
        }
        break;

    case 0xFF: // RST 38H - 4 M-cycles (call to 0x0038)
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->pc >> 8);
        mco_yield(co);

        lr35902->sp--;
        lr35902_write8(lr35902, lr35902->sp, lr35902->pc & 0xFF);
        mco_yield(co);

        lr35902->pc = 0x0038;
        break;

    default: break;
  }

    if (lr35902->remaining_cycles == 0) {
      break;
    }
  }
}

u8 lr35902_read8(lr35902* lr35902, u16 addr) {
  switch (addr) {
    case 0 ... 0x3fff: break; //rom bank 0
    case 0x4000 ... 0x7fff: break; //rom bank n
    case 0x8000 ... 0x9fff: break; //vram
    case 0xa000 ... 0xbfff: break; //external ram
    case 0xc000 ... 0xcfff: break; // work ram
    case 0xd000 ... 0xdfff: break; //work ram
    case 0xe000 ... 0xfdff: break; //echo ram
    case 0xfe00 ... 0xfe9f: break; //OAM
    case 0xfea0 ... 0xfeff: break; //unusable
    case 0xff00 ... 0xff7f: break; //IO registers
    case 0xff80 ... 0xfffe: break; //high ram
    case 0xffff: break; //interrupt enable
  }

  return 0;
}

void lr35902_write8(lr35902* lr35902, u16 addr, u8 val) {
  switch (addr) {
    case 0 ... 0x3fff: break; //rom bank 0
    case 0x4000 ... 0x7fff: break; //rom bank n
    case 0x8000 ... 0x9fff: break; //vram
    case 0xa000 ... 0xbfff: break; //external ram
    case 0xc000 ... 0xcfff: break; // work ram
    case 0xd000 ... 0xdfff: break; //work ram
    case 0xe000 ... 0xfdff: break; //echo ram
    case 0xfe00 ... 0xfe9f: break; //OAM
    case 0xfea0 ... 0xfeff: break; //unusable
    case 0xff00 ... 0xff7f: break; //IO registers
    case 0xff80 ... 0xfffe: break; //high ram
    case 0xffff: break; //interrupt enable
  }
}

void lr35902_main() {
  lr35902 lr35902;
  lr35902_new(&lr35902);
  lr35902_run(&lr35902, 10);
  lr35902_destroy(&lr35902);
}