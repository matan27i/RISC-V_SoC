# Generate RVC decompressor test vectors: tb_rvc_c.mem (16-bit C instrs)
# and tb_rvc_exp.mem (independently-encoded expected 32-bit RV32I).
def b(v,hi,lo): return (v>>lo)&((1<<(hi-lo+1))-1)
# --- RV32I encoders (expected) ---
def I(imm,rs1,f3,rd,op): return ((imm&0xFFF)<<20)|(rs1<<15)|(f3<<12)|(rd<<7)|op
def S(imm,rs2,rs1,f3,op): return ((b(imm,11,5))<<25)|(rs2<<20)|(rs1<<15)|(f3<<12)|((imm&0x1F)<<7)|op
def R(f7,rs2,rs1,f3,rd,op): return (f7<<25)|(rs2<<20)|(rs1<<15)|(f3<<12)|(rd<<7)|op
def U(imm,rd,op): return (imm&0xFFFFF000)|(rd<<7)|op
def Bt(imm,rs2,rs1,f3,op):
    return (b(imm,12,12)<<31)|(b(imm,10,5)<<25)|(rs2<<20)|(rs1<<15)|(f3<<12)|(b(imm,4,1)<<8)|(b(imm,11,11)<<7)|op
def J(imm,rd,op):
    return (b(imm,20,20)<<31)|(b(imm,10,1)<<21)|(b(imm,11,11)<<20)|(b(imm,19,12)<<12)|(rd<<7)|op
OPI,OPL,OPS,OPR,LUI,JAL,JALR,BR,SYS=0x13,0x03,0x23,0x33,0x37,0x6F,0x67,0x63,0x73

# --- RVC encoders: place imm/reg bits into 16-bit instr ---
def setb(w,pos,val,nbits=1):
    for i in range(nbits):
        if (val>>i)&1: w|=(1<<(pos+i))
    return w
def ci(f3,quad): return (f3<<13)|quad
def c_addi(rd,imm):
    w=ci(0,1); w|=rd<<7; w=setb(w,12,b(imm,5,5)); w=setb(w,2,imm&0x1F,5); return w
def c_li(rd,imm):
    w=ci(2,1); w|=rd<<7; w=setb(w,12,b(imm,5,5)); w=setb(w,2,imm&0x1F,5); return w
def c_lui(rd,imm):  # imm is full value (nzimm<<12)
    n=b(imm,17,12); w=ci(3,1); w|=rd<<7; w=setb(w,12,b(n,5,5)); w=setb(w,2,n&0x1F,5); return w
def c_addi16sp(imm):
    w=ci(3,1); w|=2<<7
    w=setb(w,12,b(imm,9,9)); w=setb(w,6,b(imm,4,4)); w=setb(w,5,b(imm,6,6))
    w=setb(w,3,b(imm,8,7),2); w=setb(w,2,b(imm,5,5)); return w
def c_addi4spn(rdp,imm):
    w=ci(0,0); w=setb(w,2,rdp-8,3)
    w=setb(w,11,b(imm,5,4),2); w=setb(w,7,b(imm,9,6),4); w=setb(w,6,b(imm,2,2)); w=setb(w,5,b(imm,3,3)); return w
def c_lw(rdp,rs1p,imm):
    w=ci(2,0); w=setb(w,2,rdp-8,3); w=setb(w,7,rs1p-8,3)
    w=setb(w,5,b(imm,6,6)); w=setb(w,10,b(imm,5,3),3); w=setb(w,6,b(imm,2,2)); return w
def c_sw(rs2p,rs1p,imm):
    w=ci(6,0); w=setb(w,2,rs2p-8,3); w=setb(w,7,rs1p-8,3)
    w=setb(w,5,b(imm,6,6)); w=setb(w,10,b(imm,5,3),3); w=setb(w,6,b(imm,2,2)); return w
def c_lwsp(rd,imm):
    w=ci(2,2); w|=rd<<7; w=setb(w,12,b(imm,5,5)); w=setb(w,4,b(imm,4,2),3); w=setb(w,2,b(imm,7,6),2); return w
def c_swsp(rs2,imm):
    w=ci(6,2); w=setb(w,2,rs2,5); w=setb(w,9,b(imm,5,2),4); w=setb(w,7,b(imm,7,6),2); return w
def c_slli(rd,sh):
    w=ci(0,2); w|=rd<<7; w=setb(w,12,b(sh,5,5)); w=setb(w,2,sh&0x1F,5); return w
def c_srli(rdp,sh):
    w=ci(4,1); w=setb(w,7,rdp-8,3); w=setb(w,10,0,2); w=setb(w,12,b(sh,5,5)); w=setb(w,2,sh&0x1F,5); return w
def c_srai(rdp,sh):
    w=ci(4,1); w=setb(w,7,rdp-8,3); w=setb(w,10,1,2); w=setb(w,12,b(sh,5,5)); w=setb(w,2,sh&0x1F,5); return w
def c_andi(rdp,imm):
    w=ci(4,1); w=setb(w,7,rdp-8,3); w=setb(w,10,2,2); w=setb(w,12,b(imm,5,5)); w=setb(w,2,imm&0x1F,5); return w
def c_rr(rdp,rs2p,sel):  # 00 sub,01 xor,10 or,11 and
    w=ci(4,1); w=setb(w,7,rdp-8,3); w=setb(w,10,3,2); w=setb(w,5,sel,2); w=setb(w,2,rs2p-8,3); return w
def c_j(imm):  return _cj(5,imm)
def c_jal(imm): return _cj(1,imm)
def _cj(f3,imm):
    w=ci(f3,1)
    w=setb(w,12,b(imm,11,11)); w=setb(w,11,b(imm,4,4)); w=setb(w,9,b(imm,9,8),2)
    w=setb(w,8,b(imm,10,10)); w=setb(w,7,b(imm,6,6)); w=setb(w,6,b(imm,7,7))
    w=setb(w,3,b(imm,3,1),3); w=setb(w,2,b(imm,5,5)); return w
def c_beqz(rs1p,imm): return _cb(6,rs1p,imm)
def c_bnez(rs1p,imm): return _cb(7,rs1p,imm)
def _cb(f3,rs1p,imm):
    w=ci(f3,1); w=setb(w,7,rs1p-8,3)
    w=setb(w,12,b(imm,8,8)); w=setb(w,10,b(imm,4,3),2); w=setb(w,5,b(imm,7,6),2)
    w=setb(w,3,b(imm,2,1),2); w=setb(w,2,b(imm,5,5)); return w
def c_jr(rs1):  w=ci(4,2); w|=rs1<<7; return w
def c_jalr(rs1): w=ci(4,2); w=setb(w,12,1); w|=rs1<<7; return w
def c_mv(rd,rs2): w=ci(4,2); w|=rd<<7; w=setb(w,2,rs2,5); return w
def c_add(rd,rs2): w=ci(4,2); w=setb(w,12,1); w|=rd<<7; w=setb(w,2,rs2,5); return w

cases=[
 (c_addi(1,1),        I(1,1,0,1,OPI),            "C.ADDI x1,1"),
 (c_addi(5,-1),       I(-1&0xFFF,5,0,5,OPI),     "C.ADDI x5,-1"),
 (c_li(10,-5),        I(-5&0xFFF,0,0,10,OPI),    "C.LI x10,-5"),
 (c_lui(8,0x3000),    U(0x3000,8,LUI),           "C.LUI x8,0x3"),
 (c_addi16sp(-32),    I(-32&0xFFF,2,0,2,OPI),    "C.ADDI16SP -32"),
 (c_addi4spn(8,16),   I(16,2,0,8,OPI),           "C.ADDI4SPN x8,16"),
 (c_lw(8,9,8),        I(8,9,2,8,OPL),            "C.LW x8,8(x9)"),
 (c_sw(8,9,12),       S(12,8,9,2,OPS),           "C.SW x8,12(x9)"),
 (c_lwsp(3,16),       I(16,2,2,3,OPL),           "C.LWSP x3,16(sp)"),
 (c_swsp(4,20),       S(20,4,2,2,OPS),           "C.SWSP x4,20(sp)"),
 (c_slli(2,3),        I(3,2,1,2,OPI),            "C.SLLI x2,3"),
 (c_srli(8,4),        I(4,8,5,8,OPI),            "C.SRLI x8,4"),
 (c_srai(9,2),        I((0x400|2),9,5,9,OPI),    "C.SRAI x9,2"),
 (c_andi(8,5),        I(5,8,7,8,OPI),            "C.ANDI x8,5"),
 (c_rr(8,9,0),        R(0x20,9,8,0,8,OPR),       "C.SUB x8,x9"),
 (c_rr(8,9,2),        R(0,9,8,6,8,OPR),          "C.OR x8,x9"),
 (c_j(-8),            J(-8&0x1FFFFF,0,JAL),      "C.J -8"),
 (c_jal(16),          J(16,1,JAL),               "C.JAL 16"),
 (c_beqz(8,-4),       Bt(-4&0x1FFF,0,8,0,BR),    "C.BEQZ x8,-4"),
 (c_bnez(9,8),        Bt(8,0,9,1,BR),            "C.BNEZ x9,8"),
 (c_jr(5),            I(0,5,0,0,JALR),           "C.JR x5"),
 (c_jalr(6),          I(0,6,0,1,JALR),           "C.JALR x6"),
 (c_mv(7,8),          R(0,8,0,0,7,OPR),          "C.MV x7,x8"),
 (c_add(7,8),         R(0,8,7,0,7,OPR),          "C.ADD x7,x8"),
]
import os
d=r"C:\Users\matan\Documents\CLOUDE\UltraCore-V"
with open(os.path.join(d,"tb_rvc_c.mem"),"w") as fc, open(os.path.join(d,"tb_rvc_exp.mem"),"w") as fe:
    for c,e,name in cases:
        fc.write("%04X\n"%(c&0xFFFF)); fe.write("%08X\n"%(e&0xFFFFFFFF))
print("generated %d vectors"%len(cases))
