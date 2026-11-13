import sys
from pathlib import Path
raw=Path(sys.argv[1]).read_bytes()
base=0
expected={}
for line in Path(sys.argv[2]).read_text().splitlines():
    if not line.strip(): continue
    record=bytes.fromhex(line.strip()[1:])
    if sum(record)&255: raise SystemExit('Invalid Intel HEX checksum')
    size=record[0]; address=int.from_bytes(record[1:3],'big'); kind=record[3]
    data=record[4:4+size]
    if kind==0: expected.update({base+address+i:b for i,b in enumerate(data)})
    elif kind==4: base=int.from_bytes(data,'big')<<16
    elif kind==2: base=int.from_bytes(data,'big')<<4
if not expected or max(expected)>=len(raw): raise SystemExit('Invalid previous firmware image')
bad=[address for address,value in expected.items() if raw[address]!=value]
if bad: raise SystemExit('Current board does not match last upload: '+str(len(bad))+' byte differences; no upload performed')
print('CURRENT_BOARD_MATCHES_PREVIOUS_UPLOAD')
