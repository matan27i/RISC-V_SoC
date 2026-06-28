# Generate tb_m_app.mem - exercises MUL/DIV/REM + forwarding after stall.
BASE = 0x2000
def lui(rd,i):   return ((i&0xFFFFF)<<12)|(rd<<7)|0x37
def addi(rd,rs1,i): return ((i&0xFFF)<<20)|(rs1<<15)|(0<<12)|(rd<<7)|0x13
def sw(rs2,rs1,i):  return (((i>>5)&0x7F)<<25)|(rs2<<20)|(rs1<<15)|(2<<12)|((i&0x1F)<<7)|0x23
def jal(rd,off):
    o=off&0x1FFFFF
    return (((o>>20)&1)<<31)|(((o>>1)&0x3FF)<<21)|(((o>>11)&1)<<20)|(((o>>12)&0xFF)<<12)|(rd<<7)|0x6F
def rtype(f7,rs2,rs1,f3,rd): return (f7<<25)|(rs2<<20)|(rs1<<15)|(f3<<12)|(rd<<7)|0x33
def add(rd,rs1,rs2): return rtype(0,rs2,rs1,0,rd)
def mul(rd,rs1,rs2): return rtype(1,rs2,rs1,0,rd)
def div(rd,rs1,rs2): return rtype(1,rs2,rs1,4,rd)
def rem(rd,rs1,rs2): return rtype(1,rs2,rs1,6,rd)

prog = [
  (lui (5,0x40000),     "x5 = peripheral base"),
  (addi(7,0,0xFF),      "x7 = 0xFF"),
  (sw  (7,5,0x34),      "GPIO DIR = outputs"),
  (addi(10,0,6),        "x10 = 6"),
  (addi(11,0,7),        "x11 = 7"),
  (mul (12,10,11),      "x12 = 6*7 = 42"),
  (addi(13,0,100),      "x13 = 100"),
  (addi(14,0,7),        "x14 = 7"),
  (div (15,13,14),      "x15 = 100/7 = 14"),
  (rem (16,13,14),      "x16 = 100%7 = 2  (back-to-back M op)"),
  (add (9,12,15),       "x9 = 42+14 = 56  (forward mul+div results)"),
  (add (9,9,16),        "x9 = 56+2 = 58 = 0x3A"),
  (sw  (9,5,0x30),      "GPIO DATA = 0x3A"),
  (jal (0,0),           "park"),
]
with open(r"C:\Users\matan\Documents\CLOUDE\UltraCore-V\tb_m_app.mem","w") as f:
    f.write("// M-extension test program (loaded at 0x2000). Auto-generated.\n")
    f.write("@%08x\n" % (BASE>>2))
    a=BASE
    for w,c in prog:
        f.write("%08X  // 0x%04X: %s\n" % (w,a,c)); a+=4
print("generated tb_m_app.mem, expected GPIO = 0x3A")
