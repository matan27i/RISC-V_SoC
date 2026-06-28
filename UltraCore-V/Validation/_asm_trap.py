# Two-pass RV32I + Zicsr assembler for the interrupt test program (scratch tool).
# Emits a $readmemh image loaded at byte 0x2000 (word offset 0x800).

BASE = 0x2000

def lui(rd, imm20):   return ((imm20 & 0xFFFFF) << 12) | (rd << 7) | 0x37
def addi(rd, rs1, i): return ((i & 0xFFF) << 20) | (rs1 << 15) | (0 << 12) | (rd << 7) | 0x13
def sw(rs2, rs1, i):  return (((i >> 5) & 0x7F) << 25) | (rs2 << 20) | (rs1 << 15) | (2 << 12) | ((i & 0x1F) << 7) | 0x23
def lw(rd, rs1, i):   return ((i & 0xFFF) << 20) | (rs1 << 15) | (2 << 12) | (rd << 7) | 0x03
def jal(rd, off):
    o = off & 0x1FFFFF
    return (((o>>20)&1)<<31) | (((o>>1)&0x3FF)<<21) | (((o>>11)&1)<<20) | (((o>>12)&0xFF)<<12) | (rd<<7) | 0x6F
def csrr(funct3, rd, csr, rs1):  # rs1 holds either a reg index or a 5-bit zimm
    return ((csr & 0xFFF) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | 0x73
def csrrw(rd, csr, rs1): return csrr(0b001, rd, csr, rs1)
def csrrs(rd, csr, rs1): return csrr(0b010, rd, csr, rs1)
MRET = 0x30200073

CSR = {'mstatus':0x300, 'mie':0x304, 'mtvec':0x305, 'mepc':0x341, 'mcause':0x342}

# Program as a flat list; labels are dicts inserted inline.
# Each entry: ('label', name) or ('op', fn_or_special, comment)
prog = []
def L(name): prog.append(('label', name))
def I(word, comment=''): prog.append(('op', word, comment))
# deferred ops needing label resolution: store a callable taking (cur_addr, labels)
def D(fn, comment=''): prog.append(('defer', fn, comment))

# ---- program ----
I(lui(5, 0x40000), "x5 = 0x40000000 (peripheral base)")
# x6 = handler address (lui hi + addi lo, with sign correction)
def load_handler_hi(cur, labels):
    a = labels['handler']; lo = a & 0xFFF; hi = (a - ((lo ^ 0x800) - 0x800)) >> 12
    return lui(6, hi & 0xFFFFF)
def load_handler_lo(cur, labels):
    a = labels['handler']; lo = a & 0xFFF
    if lo >= 0x800: lo -= 0x1000
    return addi(6, 6, lo)
D(load_handler_hi, "x6 = hi(handler)")
D(load_handler_lo, "x6 = handler")
I(csrrw(0, CSR['mtvec'], 6), "mtvec = handler")
I(addi(7, 0, 0x80), "x7 = 0x80 (MTIE)")
I(csrrs(0, CSR['mie'], 7), "mie.MTIE = 1")
I(addi(7, 0, 0x8), "x7 = 0x8 (MIE)")
I(csrrs(0, CSR['mstatus'], 7), "mstatus.MIE = 1")
I(addi(7, 0, 0xFF), "x7 = 0xFF")
I(sw(7, 5, 0x34), "GPIO DIR = outputs")
I(addi(7, 0, 0x11), "x7 = 0x11")
I(sw(7, 5, 0x30), "GPIO DATA = 0x11 (pre-int marker)")
I(addi(7, 0, 50), "x7 = 50")
I(sw(7, 5, 0x14), "TIMER COMPARE = 50")
I(addi(7, 0, 1), "x7 = 1")
I(sw(7, 5, 0x10), "TIMER CTRL.ENABLE = 1")
L('loop')
D(lambda cur, labels: jal(0, labels['loop'] - cur), "spin: jal x0, loop")
L('handler')
I(addi(8, 0, 0xCC), "x8 = 0xCC")
I(sw(8, 5, 0x30), "GPIO DATA = 0xCC (interrupt marker)")
I(addi(8, 0, 0x2), "x8 = 2 (CLEAR)")
I(sw(8, 5, 0x10), "TIMER CTRL = CLEAR (ack + stop)")
I(MRET, "mret -> return to loop")

# ---- pass 1: assign addresses ----
labels = {}
addr = BASE
for e in prog:
    if e[0] == 'label':
        labels[e[1]] = addr
    else:
        addr += 4

# ---- pass 2: emit ----
words = []
addr = BASE
for e in prog:
    if e[0] == 'label':
        continue
    if e[0] == 'op':
        words.append((addr, e[1], e[2]))
    else:  # defer
        words.append((addr, e[1](addr, labels), e[2]))
    addr += 4

# ---- write .mem ----
with open(r"C:\Users\matan\Documents\CLOUDE\UltraCore-V\tb_trap_app.mem", "w") as f:
    f.write("// Interrupt test program (loaded at 0x2000). Auto-generated.\n")
    f.write("@%08x\n" % (BASE >> 2))
    for a, w, c in words:
        f.write("%08X  // 0x%04X: %s\n" % (w, a, c))

print("labels:", {k: hex(v) for k, v in labels.items()})
for a, w, c in words:
    print("0x%04X: %08X  %s" % (a, w, c))
