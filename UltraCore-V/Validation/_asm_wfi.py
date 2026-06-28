# Generate tb_wfi_app.mem - WFI sleeps until a timer interrupt wakes/vectors.
BASE = 0x2000
def lui(rd,i):   return ((i&0xFFFFF)<<12)|(rd<<7)|0x37
def addi(rd,rs1,i): return ((i&0xFFF)<<20)|(rs1<<15)|(0<<12)|(rd<<7)|0x13
def sw(rs2,rs1,i):  return (((i>>5)&0x7F)<<25)|(rs2<<20)|(rs1<<15)|(2<<12)|((i&0x1F)<<7)|0x23
def jal(rd,off):
    o=off&0x1FFFFF
    return (((o>>20)&1)<<31)|(((o>>1)&0x3FF)<<21)|(((o>>11)&1)<<20)|(((o>>12)&0xFF)<<12)|(rd<<7)|0x6F
def csrr(f3,rd,csr,rs1): return ((csr&0xFFF)<<20)|(rs1<<15)|(f3<<12)|(rd<<7)|0x73
def csrrw(rd,csr,rs1): return csrr(1,rd,csr,rs1)
def csrrs(rd,csr,rs1): return csrr(2,rd,csr,rs1)
WFI=0x10500073
CSR={'mstatus':0x300,'mie':0x304,'mtvec':0x305}

prog=[]; D=[]
def emit(w,c): prog.append((w,c))
labels={}

# pass-1 layout is trivial here (known offsets); build with placeholders then patch.
items=[
 ('lui',5,0x40000,"x5=periph base"),
 ('hi',None,None,"x6=hi(handler)"),
 ('lo',None,None,"x6=handler"),
 ('csrrw',0,CSR['mtvec'],6,"mtvec=handler"),
 ('addi',7,0,0x80,"x7=MTIE"),
 ('csrrs',0,CSR['mie'],7,"mie.MTIE=1"),
 ('addi',7,0,0x8,"x7=MIE"),
 ('csrrs',0,CSR['mstatus'],7,"mstatus.MIE=1"),
 ('addi',7,0,0xFF,"x7=0xFF"),
 ('sw',7,5,0x34,"GPIO DIR=out"),
 ('addi',7,0,0x11,"x7=0x11"),
 ('sw',7,5,0x30,"GPIO=0x11 (awake marker)"),
 ('addi',7,0,50,"x7=50"),
 ('sw',7,5,0x14,"TIMER COMPARE=50"),
 ('addi',7,0,1,"x7=1"),
 ('sw',7,5,0x10,"TIMER CTRL.ENABLE=1"),
 ('label','sleep'),
 ('wfi',"wfi: sleep until interrupt"),
 ('jal_sleep',"jal x0, sleep"),
 ('label','handler'),
 ('addi',8,0,0xCC,"x8=0xCC"),
 ('sw',8,5,0x30,"GPIO=0xCC (woke + vectored)"),
 ('addi',8,0,0x2,"x8=2 CLEAR"),
 ('sw',8,5,0x10,"TIMER CTRL=CLEAR (ack+stop)"),
 ('mret',"mret"),
]
# layout
addr=BASE
for it in items:
    if it[0]=='label': labels[it[1]]=addr
    else: addr+=4
# emit
addr=BASE; words=[]
for it in items:
    if it[0]=='label': continue
    k=it[0]
    if k=='lui': w=lui(it[1],it[2])
    elif k=='hi':
        a=labels['handler']; lo=a&0xFFF; hi=(a-(((lo^0x800)-0x800)))>>12; w=lui(6,hi&0xFFFFF)
    elif k=='lo':
        a=labels['handler']; lo=a&0xFFF
        if lo>=0x800: lo-=0x1000
        w=addi(6,6,lo)
    elif k=='csrrw': w=csrrw(it[1],it[2],it[3])
    elif k=='csrrs': w=csrrs(it[1],it[2],it[3])
    elif k=='addi': w=addi(it[1],it[2],it[3])
    elif k=='sw': w=sw(it[1],it[2],it[3])
    elif k=='wfi': w=WFI
    elif k=='jal_sleep': w=jal(0, labels['sleep']-addr)
    elif k=='mret': w=0x30200073
    words.append((addr,w,it[-1])); addr+=4
with open(r"C:\Users\matan\Documents\CLOUDE\UltraCore-V\tb_wfi_app.mem","w") as f:
    f.write("// WFI test program. Auto-generated.\n@%08x\n"%(BASE>>2))
    for a,w,c in words: f.write("%08X  // 0x%04X: %s\n"%(w,a,c))
print("labels:",{k:hex(v) for k,v in labels.items()})
