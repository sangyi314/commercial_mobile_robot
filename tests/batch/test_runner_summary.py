import json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from summarize_runs import summarize

def test_missing_and_invalid_are_indexed_without_write(tmp_path):
 (tmp_path/'trial-missing').mkdir();p=tmp_path/'trial-invalid';p.mkdir();(p/'manifest.json').write_text('{broken')
 before={str(x):x.read_bytes() for x in tmp_path.rglob('*') if x.is_file()};report=summarize(tmp_path,'trial-');assert report['total']==2 and report['classes']['incomplete_or_missing']==1 and report['classes']['invalid']==1;assert before=={str(x):x.read_bytes() for x in tmp_path.rglob('*') if x.is_file()}
