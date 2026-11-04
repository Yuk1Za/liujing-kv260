"""Prepare PYNQ memory metadata without programming PL or invoking xclbinutil."""
import argparse,hashlib,json
from pathlib import Path
from pynq.pl_server.embedded_device import RuntimeMetadataParser,Metadata,_ip_to_topology,BLANK_METADATA
p=argparse.ArgumentParser();p.add_argument('artifact');a=p.parse_args();root=Path(a.artifact)
hwh=root/'liujing.hwh';parser=RuntimeMetadataParser(Metadata(input=hwh))
(root/'xclbin_mem.json').write_text(json.dumps(_ip_to_topology(parser.mem_dict),indent=2))
(root/'xclbin_metadata.xml').write_text(BLANK_METADATA)
(root/'xclbin_source.json').write_text(json.dumps({'hwh_sha256':hashlib.sha256(hwh.read_bytes()).hexdigest(),'scope':'PYNQ HWH-derived memory metadata; no sensor data and no PL configuration'},indent=2))
print((root/'xclbin_mem.json').read_text())
