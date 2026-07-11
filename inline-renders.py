#!/usr/bin/env python3
"""
MARGA pitch — inline ChatGPT renders into the proposal page.

Usage:
  1. Drop the ChatGPT mockup images into  mocks/renders/
     - filename becomes the caption:  entrance_monolith_at_dawn.png -> "entrance monolith at dawn"
     - end a filename with  _wide  to make it span the full page width
     - files are placed in alphabetical order; prefix 01_, 02_ ... to control it
  2. Run:  python3 inline-renders.py
  3. Deploy the generated  marga-pitch-built.html  to the existing artifact URL.

Images are resized to max 1600 px and recompressed as JPEG (sips, macOS)
so the built page stays a reasonable size. Source page is never modified.
"""
import base64, os, re, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, 'marga-pitch.html')
OUT = os.path.join(HERE, 'marga-pitch-built.html')
RENDERS = os.path.join(HERE, 'mocks', 'renders')

exts = ('.png', '.jpg', '.jpeg', '.webp')
files = sorted(f for f in os.listdir(RENDERS) if f.lower().endswith(exts)) if os.path.isdir(RENDERS) else []
if not files:
    sys.exit('no images in mocks/renders/ — nothing to inline')

entries = []
for f in files:
    stem = os.path.splitext(f)[0]
    wide = stem.endswith('_wide')
    if wide:
        stem = stem[:-5]
    cap = re.sub(r'^\d+[_ -]*', '', stem).replace('_', ' ').replace('-', ' ').strip()
    tmp = tempfile.mktemp(suffix='.jpg')
    subprocess.run(['sips', '-s', 'format', 'jpeg', '-s', 'formatOptions', '78',
                    '-Z', '1600', os.path.join(RENDERS, f), '--out', tmp],
                   check=True, capture_output=True)
    b64 = base64.b64encode(open(tmp, 'rb').read()).decode()
    os.remove(tmp)
    entries.append('{src:"data:image/jpeg;base64,%s",cap:%s,wide:%s}'
                   % (b64, repr(cap).replace("'", '"'), 'true' if wide else 'false'))
    print('  inlined %-40s %5d KB  %s' % (f, len(b64) // 1366, '(wide)' if wide else ''))

s = open(SRC, encoding='utf-8').read()
marker = 'const MOCKS=[];'
if marker not in s:
    sys.exit('marker "const MOCKS=[];" not found in marga-pitch.html')
s = s.replace(marker, 'const MOCKS=[\n' + ',\n'.join(entries) + '\n];', 1)
open(OUT, 'w', encoding='utf-8').write(s)
print('wrote %s (%d images, %d KB total)' % (OUT, len(entries), len(s) // 1024))
