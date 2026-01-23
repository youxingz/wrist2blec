import sys

UICR_START = 0x10001000
UICR_END   = 0x10002000  # 覆盖整段 UICR 区域即可

def parse_record(line: str):
    # Intel HEX: :LLAAAATT[DD...]CC
    if not line.startswith(':'):
        return None
    line = line.strip()
    ll = int(line[1:3], 16)
    addr = int(line[3:7], 16)
    rtype = int(line[7:9], 16)
    data = bytes.fromhex(line[9:9+ll*2]) if ll else b""
    return ll, addr, rtype, data

def main(inp, outp):
    ela = 0  # extended linear address (upper 16 bits)
    kept = 0
    dropped = 0

    with open(inp, 'r', encoding='utf-8', errors='ignore') as f, open(outp, 'w', encoding='utf-8') as g:
        for raw in f:
            rec = parse_record(raw)
            if rec is None:
                # 非 HEX 行原样写
                g.write(raw)
                continue

            ll, addr, rtype, data = rec

            if rtype == 0x04 and ll == 2:
                # ELA record: upper 16 bits
                ela = int.from_bytes(data, 'big')
                g.write(raw)
                kept += 1
                continue

            if rtype == 0x00:
                # data record
                abs_addr = (ela << 16) | addr
                # 一个 record 可能跨界，保守处理：只要有任何部分落在 UICR 就整条丢弃
                if abs_addr < UICR_END and (abs_addr + ll) > UICR_START:
                    dropped += 1
                    continue

            # 其它记录保留
            g.write(raw)
            kept += 1

    print(f"OK. kept={kept}, dropped={dropped}, out={outp}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 strip_uicr_hex.py in.hex out.hex")
        sys.exit(2)
    main(sys.argv[1], sys.argv[2])
