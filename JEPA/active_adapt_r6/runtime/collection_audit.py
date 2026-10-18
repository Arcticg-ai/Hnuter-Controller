"""R6 additional data/online checks; no performance-based repeat or gate relaxation."""
import json,math
import numpy as np,pandas as pd
from excitation_r6 import reference
def audit(p):
    c={};d={}
    try:
        case=next(x for x in json.loads((p.parents[1]/'cases.json').read_text()) if x['id']==p.name);mode=case['excitation'];cmds=[json.loads(s) for s in (p/'commands_causal.jsonl').read_text().splitlines()];active=[x for x in cmds if x['phase']=='EVALUATION' and 49<x['elapsed']<67];errors=[np.max(abs(np.array(x['actual_jerk'][:2])-reference(x['elapsed'],mode)[0][:2])) for x in active];c['excitation_published']=len(active)>100 and max(errors)<1e-6;d['excitation_error']=max(errors)
        summary=json.loads((p/'online_summary.json').read_text());c['online_completed']=summary['completed'] and not summary['partial_line'];c['frozen_backbone']=summary['frozen_backbone_files_unchanged'];c['no_online_control']=not summary['control_enabled'];c['online_coverage']=summary['intervals']>600;c['alignment_no_rejects']=summary['alignment_rejects']==0
        z=pd.read_csv(p/'truth_diagnostic.csv',usecols=['t','dt','iteration','stage']);z=z[z.stage==1];c['truth_100hz_1ms_physics']=bool(len(z)>5000 and np.max(abs(z.dt-.001))<1e-12 and np.all(z.iteration%10==0) and np.max(np.diff(z.t))<.011)
        rows=[json.loads(s) for s in (p/'online_predictions.jsonl').read_text().splitlines()];end_elapsed={x['t_us']:x['elapsed'] for x in [json.loads(s) for s in (p/'aligned_states.jsonl').read_text().splitlines()]};c['update_only_id_or_trim']=all((34<=end_elapsed[x['end_us']]<44) if x['update_reason']=='trim' else (48<=end_elapsed[x['end_us']]<56) if x['update_reason']=='updated' else True for x in rows);c['causal_target_before_update']=all(x['end_us']-x['t_us']==100000 and x['processed_ns']>=x['received_through_ns'] for x in rows);d['latency_p95_ms']=float(np.quantile([(x['processed_ns']-x['received_through_ns'])*1e-6 for x in rows if x['elapsed']>18],.95));d['core_p95_ms']=float(np.quantile([x['core_ms'] for x in rows],.95));c['latency_budget']=d['latency_p95_ms']<=250 and d['core_p95_ms']<=10
        receipts=[json.loads(s) for s in (p/'adapt_receipts.jsonl').read_text().splitlines()];d['callback_intervals_p99_s']={k:float(np.quantile(np.diff([x['t_us'] for x in receipts if x['key']==k])*1e-6,.99)) for k in ['motor','servo','position','attitude','rate']};c['actuator_callback_coverage']=all(d['callback_intervals_p99_s'][k]<.03 for k in ['motor','servo'])
    except Exception as e:c['collection_exception_free']=False;d['error']=repr(e)
    return dict(checks={k:bool(v) for k,v in c.items()},details=d,passed=all(c.values()))
