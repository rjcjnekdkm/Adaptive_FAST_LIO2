import csv, math, statistics as st
from pathlib import Path
B=Path('/home/romi/Adaptive_FAST_LIO2/experiments/adaptive_map_v1/validation')
def read(p):
 with p.open() as f:return list(csv.DictReader(f))
for seq in ('geode_tunnel5','ntu_spms2','geode_waterway_short'):
 a=read(next((B/seq/'ours_on/run01').glob('runtime*.csv')))
 offdir=B/seq/('ours_off/run02' if seq=='geode_tunnel5' else 'ours_off/run01')
 b=read(sorted(offdir.glob('runtime*.csv'))[0]); db={x['lidar_end_time']:x for x in b}; t0=float(a[0]['lidar_end_time'])
 pairs=[(x,db[x['lidar_end_time']]) for x in a if x['lidar_end_time'] in db]
 print('\nSEQUENCE',seq,'matched',len(pairs),'rows',len(a),len(b))
 for k in ('sync_imu_samples','sync_imu_first_time','sync_imu_last_time','lidar_begin_time'):
  bad=[(float(x['lidar_end_time'])-t0,x[k],y[k]) for x,y in pairs if x[k]!=y[k]]
  print('input',k,'mismatches',len(bad),'first',bad[:5])
 for threshold in (.1,.5,1,5,10,100):
  hit=next(((float(x['lidar_end_time'])-t0,x['frame']) for x,y in pairs if math.dist([float(x['pos_'+k]) for k in 'xyz'],[float(y['pos_'+k]) for k in 'xyz'])>threshold),None)
  print('raw difference >',threshold,hit)
 for k in ('degenerate','quality_rejected','invalid_quality_rejected','direction_rejected','persistent_quota_rejected','novel_rejected'):
  non=[x for x in a if float(x[k])>0]; print(k,'sum',sum(float(x[k]) for x in a),'first',float(non[0]['lidar_end_time'])-t0 if non else None)
 windows= [(0,100),(100,125),(125,130),(130,135),(135,140),(140,145),(145,150),(150,160)] if seq=='geode_tunnel5' else [(i,i+50) for i in range(0,int(float(a[-1]['lidar_end_time'])-t0),50)]
 for lo,hi in windows:
  print('window',lo,hi)
  for name,rs in [('on',a),('off',b)]:
   w=[x for x in rs if lo<=float(x['lidar_end_time'])-t0<hi]
   if not w:continue
   keys=('effective_points','residual_mean','map_added','map_size','quality_rejected','invalid_quality_rejected','degenerate','condition_number')
   print(name,len(w),{k:round(st.mean(float(x[k]) for x in w),3) for k in keys})
 gaps=[(float(y['lidar_end_time'])-t0,float(y['lidar_end_time'])-float(x['lidar_end_time'])) for x,y in zip(a,a[1:]) if float(y['lidar_end_time'])-float(x['lidar_end_time'])>.15]
 print('on gaps',gaps[:25])
