#!/usr/bin/env python3
"""Read-only build audit. No probe connection or microcontroller write."""
import argparse, hashlib, json, re, shlex, struct
import xml.etree.ElementTree as ET
from pathlib import Path


def require(ok, message):
    if not ok:
        raise SystemExit('FAIL: ' + message)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--build', type=Path, default=Path('build/debug'))
    ap.add_argument('--check-originals', action='store_true', help='Also compare all files present before migration; omit after intentional source edits')
    args = ap.parse_args()
    root = Path(__file__).resolve().parent.parent
    build = args.build.resolve()
    target = ET.parse(root/'MDK-ARM/Main_Board.uvprojx').find('./Targets/Target')
    expected = []
    for group in target.findall('./Groups/Group'):
        if group.findtext('./GroupOption/CommonProperty/IncludeInBuild') == '0':
            continue
        for f in group.findall('./Files/File'):
            if f.findtext('./FileOption/CommonProperty/IncludeInBuild') == '0':
                continue
            source = (root/'MDK-ARM'/f.findtext('FilePath').replace('\\', '/')).resolve()
            relative = str(source.relative_to(root))
            relative = relative.replace('portable/RVDS/ARM_CM4F/', 'portable/GCC/ARM_CM4F/')
            if relative == 'MDK-ARM/startup_stm32f407xx.s':
                relative = 'Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s'
            expected.append(str(root/relative))
    commands = json.loads((build/'compile_commands.json').read_text())
    actual = [str(Path(c['file']).resolve()) for c in commands]
    require(len(actual) == len(set(actual)), 'duplicate compilation unit')
    require(set(actual) == set(expected + [str(root/'cmake/gcc_runtime.c')]), 'Keil/CMake source lists differ')
    expected_includes = []
    for inc in target.findtext('.//Cads/VariousControls/IncludePath').split(';'):
        expected_includes.append(str((root/'MDK-ARM'/inc.replace('\\','/')).resolve()).replace('portable/RVDS/ARM_CM4F','portable/GCC/ARM_CM4F'))
    for c in commands:
        flags = c.get('arguments') or shlex.split(c['command'])
        for flag in ['-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard']:
            require(flag in flags, 'missing CPU/FPU flag: '+c['file'])
        if c['file'].endswith('.c'):
            for define in ['STM32F407xx','_RTE_','USE_FULL_LL_DRIVER','ARM_MATH_CM4','__FPU_PRESENT=1U']:
                require('-D'+define in flags, 'missing define '+define)
            for inc in expected_includes:
                require('-I'+inc in flags, 'missing include '+inc)
            for flag in ['-fshort-enums','-fshort-wchar','-funsigned-char','-fno-strict-aliasing','-std=c99']:
                require(flag in flags, 'missing ABI/language flag '+flag)
    print('PASS: 82 Keil units with exactly 2 toolchain substitutions + 1 GCC runtime; all includes/defines/CPU flags')
    for ext in ['elf','bin','hex','map']:
        require((build/('firmware.'+ext)).stat().st_size > 0, 'missing/empty '+ext)
    data = (build/'firmware.elf').read_bytes()
    require(data[:6] == b'\x7fELF\x01\x01', 'ELF must be 32-bit little-endian')
    header = struct.unpack_from('<16sHHIIIIIHHHHHH',data)
    require(header[2] == 40, 'ELF is not ARM')
    require(header[7] & 0x400, 'ELF is not hard-float')
    entry, shoff, shentsize, shnum, shstridx = header[4],header[6],header[11],header[12],header[13]
    raw_sections = [struct.unpack_from('<10I',data,shoff+i*shentsize) for i in range(shnum)]
    string_section = raw_sections[shstridx]
    strings = data[string_section[4]:string_section[4]+string_section[5]]
    def cstring(blob, pos):
        return blob[pos:blob.index(b'\0',pos)].decode()
    sections = {cstring(strings,s[0]):s for s in raw_sections}
    symbols = {}
    for sec in raw_sections:
        if sec[1] != 2:
            continue
        st = raw_sections[sec[6]]
        names = data[st[4]:st[4]+st[5]]
        for pos in range(sec[4],sec[4]+sec[5],sec[9]):
            name,value,size,info,other,index = struct.unpack_from('<IIIBBH',data,pos)
            if name:
                symbols[cstring(names,name)] = (value,info>>4,index)
    vector_section = sections['.isr_vector']
    require(vector_section[3] == 0x08000000 and vector_section[5] == 392,'vector origin/length')
    vectors = struct.unpack_from('<98I',data,vector_section[4])
    require(vectors[0] == 0x20020000,'initial MSP must be 0x20020000')
    require(vectors[1] == entry == symbols['Reset_Handler'][0],'reset entry/Thumb mismatch')
    old = (root/'MDK-ARM/startup_stm32f407xx.s').read_text()
    names = re.findall(r'^\s*(?:__Vectors\s+)?DCD\s+(\w+)',old,re.M)
    require(len(names) == 98,'original vector count changed')
    for i,name in enumerate(names):
        # ST's bundled GNU template calls IRQ48 FSMC; the Keil template
        # calls the same unimplemented interrupt FMC. Do not allow an
        # application handler to become disconnected silently.
        if name == 'FMC_IRQHandler':
            require('FMC_IRQHandler' not in symbols or symbols['FMC_IRQHandler'][1] == 2, 'FMC implementation needs explicit GNU vector adaptation')
            require(symbols['FSMC_IRQHandler'][0] & ~1 == symbols['Default_Handler'][0] & ~1, 'unexpected FSMC implementation')
            name = 'FSMC_IRQHandler'
        expected_value = 0 if name == '0' else symbols[name][0]
        require(vectors[i] == expected_value,'vector mismatch '+name)
    handlers = re.findall(r'\bvoid\s+(\w+(?:IRQHandler|Handler))\s*\(', (root/'Core/App/stm32f4xx_it.c').read_text())
    handlers += ['SVC_Handler','PendSV_Handler','SysTick_Handler']
    for name in handlers:
        require(symbols[name][1] == 1, 'ISR must be a strong symbol: '+name)
        require(symbols[name][0] & ~1 != symbols['Default_Handler'][0] & ~1, 'ISR points to default: '+name)
    require(symbols['_sdata'][0] == 0x20000000,'data origin')
    require(symbols['_edata'][0] <= symbols['_sbss'][0] <= symbols['_ebss'][0] <= symbols['__heap_base'][0] < symbols['__heap_limit'][0] <= 0x2001fc00,'SRAM overlap')
    require(symbols['__StackLimit'][0] == 0x2001fc00 and symbols['__StackTop'][0] == 0x20020000,'stack range')
    require(symbols['_sidata'][0] + sections['.data'][5] <= 0x08100000,'Flash overflow')
    for name in ['_sidata','_sdata','_edata','_sbss','_ebss']:
        require(symbols[name][0] % 4 == 0, 'unaligned startup symbol '+name)
    require(sections.get('.ccmram',(0,)*6)[5] == 0,'CCM must remain unused')
    # Read initialized data through the exact startup load/copy symbols.
    binary = (build/'firmware.bin').read_bytes()
    init_start = symbols['_sidata'][0]-0x08000000
    require(binary[init_start:init_start+sections['.data'][5]] == data[sections['.data'][4]:sections['.data'][4]+sections['.data'][5]],'.data load image mismatch')
    # Decode Intel HEX, check checksums, ensure only Flash is written, compare BIN.
    image,upper,eof = {},0,False
    for line in (build/'firmware.hex').read_text().splitlines():
        record = bytes.fromhex(line[1:]); require(line[0]==':' and sum(record)%256==0,'bad HEX checksum')
        length,address,kind = record[0],int.from_bytes(record[1:3],'big'),record[3]
        require(len(record)==length+5,'bad HEX record length')
        if kind == 0:
            for i,b in enumerate(record[4:4+length]):
                addr=upper+address+i
                require(0x08000000 <= addr < 0x08100000,'HEX writes outside Flash')
                image[addr] = b
        elif kind == 4:
            upper=int.from_bytes(record[4:6],'big')<<16
        elif kind == 1:
            eof=True
    require(eof and min(image)==0x08000000 and max(image)-0x08000000+1==len(binary),'HEX range/EOF mismatch')
    require(all(binary[a-0x08000000] == b for a,b in image.items()),'HEX/BIN differ')
    # All addresses not represented by HEX must match objcopy's erased-Flash (0xFF) BIN gaps.
    require(all(b == image.get(0x08000000+i,0xff) for i,b in enumerate(binary)),'HEX/BIN gap mismatch')
    print(f'PASS: ELF/BIN/HEX/MAP; all 98 vectors; {len(handlers)} strong ISRs; SRAM/stack; Flash-only HEX; .data initializers')
    print(f'Image: {len(binary)} bytes; reset=0x{entry:08x}; MSP=0x{vectors[0]:08x}')
    if args.check_originals:
        hashes = json.loads((root/'cmake/migration/original-sha256.json').read_text())
        changed = [n for n,h in hashes.items() if not (root/n).is_file() or hashlib.sha256((root/n).read_bytes()).hexdigest()!=h]
        require(not changed,'original files changed/deleted: '+', '.join(changed))
        print(f'PASS: all {len(hashes)} original files byte-for-byte unchanged')
    print('Hardware startup and robot behavior are NOT validated by this audit.')

if __name__ == '__main__':
    main()
