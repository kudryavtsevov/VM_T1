#!/usr/bin/env python3
"""Generate private ESP-NOW pairing header for both builds. No keys in stdout."""
import argparse
from pathlib import Path
import re
import secrets
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--timer', required=True, help='Timer Wi-Fi STA MAC from serial monitor')
parser.add_argument('--ground', required=True, help='Ground Wi-Fi STA MAC (not AP MAC)')
a = parser.parse_args()
def mac(s):
    if not re.fullmatch(r'(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}',s):
        parser.error('Expected MAC as AA:BB:CC:DD:EE:FF')
    b=bytes.fromhex(s.replace(':',''))
    if not any(b) or b[0]&1:
        parser.error('Expected nonzero unicast MAC')
    return b

timer,ground=mac(a.timer),mac(a.ground)
if timer==ground:
    parser.error('Devices must have different STA MACs')
out=Path(__file__).parent/'common'/'Pairing.local.h'
if out.exists():
    parser.error('Pairing.local.h already exists. Back it up and remove it explicitly to re-pair.')
def array(name,b):
    return f'static const uint8_t {name}[{len(b)}] = {{'+','.join(f'0x{x:02x}' for x in b)+'};\n'
password=secrets.token_urlsafe(12)
text='#pragma once\n#include <stdint.h>\n'
for name,b in [('TIMER_MAC',timer),('GROUND_MAC',ground),('RADIO_PMK',secrets.token_bytes(16)),('RADIO_LMK',secrets.token_bytes(16))]:
    text+=array(name,b)
text+=f'#define F1Q_AP_PASSWORD "{password}"\n'
with out.open('x') as f:
    f.write(text)
out.chmod(0o600)
print('Created private F1Q/common/Pairing.local.h. Read AP password there; rebuild BOTH devices.')
