"""Evidence-preserving journal figure atlas; no smoothing or invented repeats."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil

os.environ.setdefault('MPLCONFIGDIR','/tmp/hnuter-publication-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.lines import Line2D
from matplotlib.colors import Normalize
import numpy as np
import pandas as pd

REPO=Path('/home/hnuter/px4_ws_ros2_consolidated/reports')
BASE=REPO/'nda_baseline_validation_20261011'
REV=BASE/'revised_maneuvers'
FIX=REPO/'first_roll_diagnosis_20261012/evidence'
METHODS=['direct','paper_nda','full','no_preview','no_horizon','no_nearfield']
LABELS=dict(zip(METHODS,['Direct','NDA adaptation','Full DRCDA','No preview','One step','No near-term']))
COLORS=dict(zip(METHODS,['#377EB8','#D58A00','#007F73','#B55283','#7062A3','#656565']))
STYLES=dict(zip(METHODS,['--','-.','-',':',(0,(5,2,1,2)),(0,(2,2))]))
LEGACY='Historical sensor configuration; not a post-fix algorithm ranking.'
manifest={}
OUT=None


def read(path):
    path=Path(path)
    key=str(path)
    if not path.exists():
        index=json.loads((OUT/'sources.json').read_text())
        path=OUT/index[key]['copy']
    blob=path.read_bytes();digest=hashlib.sha256(blob).hexdigest()
    target=OUT/'inputs'/f'{digest[:14]}_{path.name}'
    target.parent.mkdir(exist_ok=True)
    if not target.exists():shutil.copy2(path,target)
    manifest[key]={'sha256':digest,'copy':str(target.relative_to(OUT))}
    return pd.read_csv(path,low_memory=False)


def trace(scenario,method):
    d=read(REV/'data'/scenario/'repeat_1'/method/'diagnostic.csv')
    d=d[~d.switch_phase.isin(['idle','ingress','settle_a'])].copy()
    d['t']=d.time_s-d.time_s.iloc[0]
    actual=d[['position_x_enu_m','position_y_enu_m','position_z_rel_m']].values
    desired=d[['target_x_enu_m','target_y_enu_m','target_z_rel_m']].values
    d['ep']=np.linalg.norm(actual-desired,axis=1)
    return d


def style(name):
    plt.rcdefaults()
    plt.rcParams.update({'font.family':'DejaVu Serif' if name=='TRO' else 'DejaVu Sans',
        'font.size':8,'axes.labelsize':8,'axes.titlesize':9,'xtick.labelsize':7,
        'ytick.labelsize':7,'legend.fontsize':7,'axes.linewidth':.6,
        'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,
        'grid.color':'#DCE1E5','grid.linewidth':.45,'grid.alpha':.65,
        'lines.linewidth':1.2,'pdf.fonttype':42,'ps.fonttype':42,
        'svg.fonttype':'none','savefig.facecolor':'white','mathtext.fontset':'dejavuserif' if name=='TRO' else 'dejavusans'})


def frame(title,subtitle,footer,shape=(2,2),height=5.8):
    fig,axes=plt.subplots(*shape,figsize=(7.2,height),squeeze=False)
    fig.subplots_adjust(left=.12,right=.96,bottom=.18,top=.75,hspace=.72,wspace=.45)
    fig.text(.055,.962,title,fontsize=12,fontweight='bold',va='top')
    fig.text(.055,.918,subtitle,fontsize=7.5,color='#50575D',va='top')
    fig.text(.055,.025,footer,fontsize=6.5,color='#50575D',va='bottom')
    return fig,axes


def panel(ax,letter,title):
    ax.set_title(f'{letter}  {title}',loc='left',pad=7,fontweight='bold')


def legend(fig,methods):
    fig.legend([Line2D([],[],color=COLORS[m],ls=STYLES[m],lw=1.5) for m in methods],
        [LABELS[m] for m in methods],loc='upper center',bbox_to_anchor=(.53,.882),
        ncol=3 if len(methods)>3 else len(methods),frameon=False,columnspacing=1.8,handlelength=2.5)


def curve(ax,d,x,y,m,**kwargs):
    ax.plot(d[x],d[y],color=COLORS[m],ls=STYLES[m],**kwargs)


def lateral():
    d=read(BASE/'all_metrics.csv');d=d[d.cohort=='comparison_common']
    methods=METHODS[:5]
    fig,axs=frame('Rapid lateral reversal','Matched comparison and ablation | 1.2 m displacement | n = 2 runs per condition',
        LEGACY+'\nSymbols = runs; lines = mean; whiskers = observed range, not CI. Panels a-c use log scales.')
    specs=[('position_error_rms_m','Position RMSE [m]'),('attitude_error_peak_deg','Peak attitude error [deg]'),
           ('yaw_rate_peak_rad_s','Peak yaw rate [rad/s]'),('joint_tracking_error_rms_rad','Joint tracking RMSE [rad]')]
    for ax,(field,ylabel),letter in zip(axs.flat,specs,'abcd'):
        for i,m in enumerate(methods):
            s=d[d.method==m];g=s.groupby('duration')[field]
            means=g.mean();x=means.index.to_numpy();offs=(i-2)*.012
            ax.errorbar(x+offs,means,yerr=[means-g.min(),g.max()-means],color=COLORS[m],
                ls=STYLES[m],lw=1,marker='o',ms=3,capsize=2)
            for t,group in s.groupby('duration'):
                ax.scatter(t+offs+np.array([-.006,.006]),group[field],s=13,facecolors='white',
                    edgecolors=COLORS[m],linewidths=.7,zorder=4)
        panel(ax,letter,ylabel.split(' [')[0]);ax.set(xlabel='Transfer time T [s]',ylabel=ylabel,xticks=[1.1,1.5,2.0])
        if letter!='d':ax.set_yscale('log')
        else:ax.set_ylim(bottom=0)
    legend(fig,methods)
    d.to_csv(OUT/'lateral_metrics.csv',index=False)
    return fig


def lissajous():
    methods=METHODS[:3];data={m:trace('lissajous_7',m) for m in methods}
    fig,axs=frame('Three-dimensional Lissajous tracking','T = 7 s with a shared 0.75 s phase ramp | one run per method',
        LEGACY+'\nTracking errors include the 5 s final hold; 3D paths show motion only. Reference: black dotted line.',height=5.9)
    spec=axs[0,0].get_subplotspec();axs[0,0].remove();ax3=fig.add_subplot(spec,projection='3d')
    ref=data['full'];r=ref[ref.t<=7.75]
    xyz=['position_x_enu_m','position_y_enu_m','position_z_rel_m']
    target=['target_x_enu_m','target_y_enu_m','target_z_rel_m']
    ax3.plot(*[r[c] for c in target],color='#222222',ls=':',lw=1)
    for m,d in data.items():
        s=d[d.t<=7.75];ax3.plot(*[s[c] for c in xyz],color=COLORS[m],ls=STYLES[m],lw=1)
        curve(axs[0,1],s,xyz[0],xyz[1],m)
        curve(axs[1,0],d,'t','ep',m)
        curve(axs[1,1],d,'t','attitude_error_angle_deg',m)
    ax3.set(xlabel='x [m]',ylabel='y [m]',zlabel='z [m]');ax3.view_init(25,-60)
    ax3.tick_params(pad=0,labelsize=6);ax3.xaxis.labelpad=0;ax3.yaxis.labelpad=0;ax3.zaxis.labelpad=0
    span=np.ptp(r[target].values,axis=0);ax3.set_box_aspect(np.maximum(span,.5))
    axs[0,1].plot(r[target[0]],r[target[1]],color='#222222',ls=':',lw=1)
    axs[0,1].set(xlabel='x [m]',ylabel='y [m]');axs[0,1].set_aspect('equal',adjustable='datalim')
    for ax,ylabel in [(axs[1,0],'Position error [m]'),(axs[1,1],'SO(3) error [deg]')]:
        ax.axvspan(7.75,12.75,color='#CDD1D6',alpha=.2,zorder=0)
        ax.set(xlabel='Time from motion start [s]',ylabel=ylabel,xlim=(0,12.75),ylim=(0,None))
    for ax,letter,title in zip([ax3,axs[0,1],axs[1,0],axs[1,1]],'abcd',
        ['Spatial path','Horizontal projection','Position tracking','Attitude tracking']):panel(ax,letter,title)
    legend(fig,methods)
    return fig


def ablation():
    d=read(REV/'metrics.csv');d=d[d.scenario=='lissajous_7'].set_index('method')
    methods=['full','no_preview','no_horizon','no_nearfield']
    fig,axs=frame('Prediction and allocation ablations','7 s Lissajous | identical outer-loop parameters and actuator limits',
        LEGACY+'\nOne run per variant. ECDF describes samples within a run, not independent trials. TV is not energy.')
    fig.subplots_adjust(left=.17,wspace=.75)
    for ax,field,title,unit,letter in [(axs[0,0],'position_rms_m','Position RMSE','m','a'),
        (axs[0,1],'so3_peak_deg','Peak attitude error','deg','b')]:
        for i,m in enumerate(methods):
            value=float(d.loc[m,field]);ax.hlines(i,0,value,color=COLORS[m],lw=2)
            ax.scatter(value,i,s=27,color=COLORS[m],zorder=3)
            ax.annotate(f'{value:.3f}' if unit=='m' else f'{value:.2f}',(value,i),xytext=(5,0),
                textcoords='offset points',va='center',fontsize=7)
        ax.set(yticks=range(4),yticklabels=[LABELS[m] for m in methods],xlabel=f'{title} [{unit}]')
        ax.invert_yaxis();ax.set_xlim(0,float(d.loc[methods,field].max())*1.3);panel(ax,letter,title)
    for m in methods:
        s=trace('lissajous_7',m);v=np.sort(s.ep.values)
        axs[1,0].plot(v,np.arange(1,len(v)+1)/len(v),color=COLORS[m],ls=STYLES[m])
    axs[1,0].set(xlabel='Position error [m]',ylabel='Empirical cumulative probability',ylim=(0,1.03))
    panel(axs[1,0],'c','Within-run error distribution')
    ax=axs[1,1]
    for i,m in enumerate(methods):
        values=d.loc[m,['commanded_joint_total_variation_rad','measured_joint_total_variation_rad']].astype(float)
        ax.plot(values,[i,i],color=COLORS[m],lw=1)
        ax.scatter(values.iloc[0],i,color=COLORS[m],s=27,marker='o')
        ax.scatter(values.iloc[1],i,edgecolors=COLORS[m],facecolors='white',s=27,marker='s',zorder=4)
    ax.set(yticks=range(4),yticklabels=[LABELS[m] for m in methods],xlabel='Sum of joint total variation [rad]',xscale='log')
    ax.invert_yaxis();panel(ax,'d','Actuator activity')
    ax.text(.98,.98,'Filled: command\nOpen: measured',transform=ax.transAxes,fontsize=6.5,color='#50575D',va='top',ha='right')
    legend(fig,methods)
    return fig


def stress():
    methods=['direct','paper_nda','full','no_horizon'];metrics=read(REV/'metrics.csv')
    fig,axs=frame('Aggressive-flight stress test','T = 5 s with a shared 0.75 s ramp | failures retained, not ranked by post-crash RMSE',
        LEGACY+'\nMarkers: first true height below 0.5 m; not the onset of all instability. Motion ends at 5.75 s.',height=5.8)
    for m in methods:
        d=trace('lissajous_5',m)
        truth=read(REV/'data/lissajous_5/repeat_1'/m/'ulog_flight_diagnostics.csv')
        truth=truth[truth.time_from_motion_s.between(0,10.75)]
        curve(axs[0,0],d,'t','ep',m)
        curve(axs[0,1],d,'t','attitude_error_angle_deg',m)
        curve(axs[1,0],truth,'time_from_motion_s','truth_relative_height_m',m)
        row=metrics[(metrics.scenario=='lissajous_5')&(metrics.method==m)].iloc[0]
        loss=row.first_physical_altitude_loss_s
        i=methods.index(m);ax=axs[1,1]
        end=10.75 if pd.isna(loss) else loss
        ax.hlines(i,0,end,color=COLORS[m],lw=3)
        ax.scatter(end,i,color=COLORS[m],marker='>' if pd.isna(loss) else 'X',s=38,zorder=3)
        ax.text(end-.15,i-.22,'airborne to end' if pd.isna(loss) else f'{loss:.2f} s',ha='right',fontsize=7)
        if pd.notna(loss):axs[1,0].scatter(loss,.5,marker='X',color=COLORS[m],s=32,zorder=5)
    for ax,ylabel in [(axs[0,0],'Position error [m]'),(axs[0,1],'SO(3) error [deg]'),(axs[1,0],'True relative height [m]')]:
        ax.set(xlabel='Time from motion start [s]',ylabel=ylabel,xlim=(0,10.75))
        ax.axvline(5.75,color='.45',lw=.7,ls=':')
    axs[0,0].set_ylim(bottom=0);axs[0,1].set_ylim(0,185)
    axs[1,0].axhline(.5,color='.25',lw=.7,ls='--')
    axs[1,1].set(xlim=(0,11.1),ylim=(-.6,3.6),yticks=range(4),yticklabels=[LABELS[m] for m in methods],xlabel='Observation duration [s]')
    axs[1,1].invert_yaxis()
    for ax,letter,title in zip(axs.flat,'abcd',['Position error: full range','Attitude error: full range','Independent height evidence','Height-loss event timeline']):panel(ax,letter,title)
    legend(fig,methods)
    return fig


def fix_frames():
    return [read(FIX/n/'aligned.csv') for n in ['before','after_sensor_only','after_both','final_entry_analysis']]


def diagnosis():
    data=fix_frames();labels=['Before fixes','Sensor fix only','Sensor + gain fix','Final-entry repeat']
    colors=['#AA4949','#C58A20','#007F73','#377EB8'];styles=['--','-.','-',':']
    fig,axs=frame('First positive roll: isolating the cause','15 s outbound to +80 deg, followed by 5 s hold | unchanged outer-loop gains',
        'Sequential diagnostic interventions, not allocator-method comparisons. No synthetic or smoothed samples.\nControl error uses EKF; dashed curves in c compare Gazebo truth with the reference.',height=5.9)
    for d,label,color,ls in zip(data,labels,colors,styles):
        s=d[d.motion_time.between(0,20)]
        for ax,col in [(axs[0,0],'est_truth_deg'),(axs[0,1],'attitude_error_angle_deg')]:
            ax.plot(s.motion_time,s[col],color=color,ls=ls,label=label,lw=1)
    for d,label,color,ls in zip(data[2:],labels[2:],colors[2:],styles[2:]):
        s=d[d.motion_time.between(0,20)]
        axs[1,0].plot(s.motion_time,s.attitude_error_angle_deg,color=color,ls=ls,lw=1)
        axs[1,0].plot(s.motion_time,s.truth_ref_deg,color=color,ls='--',lw=.8,alpha=.7)
    for i,(d,label,color) in enumerate(zip(data,labels,colors)):
        s=d[d.switch_phase=='roll_pos_out'];y=s.attitude_error_angle_deg
        rms=np.sqrt(np.mean(y*y));peak=y.max()
        axs[1,1].plot([rms,peak],[i,i],color=color,lw=1.4)
        axs[1,1].scatter(rms,i,color=color,s=22)
        axs[1,1].scatter(peak,i,edgecolors=color,facecolors='white',s=24,marker='D')
    axs[1,1].set(yticks=range(4),yticklabels=['Before','Sensor only','Both fixes','Repeat'],xscale='log',xlabel='Outbound SO(3) error [deg]')
    axs[1,1].invert_yaxis()
    for ax,ylabel in [(axs[0,0],'EKF versus truth [deg]'),(axs[0,1],'Control SO(3) error [deg]'),(axs[1,0],'SO(3) error [deg]')]:
        ax.set(xlabel='Time from first positive roll [s]',ylabel=ylabel,xlim=(0,20),ylim=(0,None))
        ax.axvspan(15,20,color='#CDD1D6',alpha=.18,zorder=0)
    for ax,letter,title in zip(axs.flat,'abcd',['Estimator inconsistency','Controller error, full range','Corrected runs (zoom)','Filled = RMS; diamond = peak']):panel(ax,letter,title)
    fig.legend([Line2D([],[],color=c,ls=s) for c,s in zip(colors,styles)],labels,
        loc='upper center',bbox_to_anchor=(.53,.866),ncol=2,frameon=False)
    return fig


def full_attitude():
    data=fix_frames()[2:];fig,axs=frame('Large-attitude validation after both fixes',
        '+80 roll / +180 pitch / -80 roll / -180 pitch | two sequential verification runs',
        'No-delay SITL, direct motor/servo control. Remaining negative-pitch transients are retained.\nHeatmap reports hold-segment RMS per run, not a population estimate.',height=6.0)
    colors=['#007F73','#377EB8']
    for d,c in zip(data,colors):
        s=d[d.motion_time.between(0,160)]
        for ax,col in [(axs[0,0],'attitude_error_angle_deg'),(axs[0,1],'position_error_m'),(axs[1,0],'truth_relative_height_m')]:
            ax.plot(s.motion_time,s[col],color=c,lw=1)
    for ax,ylabel in [(axs[0,0],'Control SO(3) [deg]'),(axs[0,1],'Position error [m]'),(axs[1,0],'True relative height [m]')]:
        ax.set(xlabel='Time from motion start [s]',ylabel=ylabel,xlim=(0,160),xticks=[0,40,80,120,160])
        ax.axvspan(120,160,color='#D6B480',alpha=.15,zorder=0)
        for x in [40,80,120]:ax.axvline(x,color='.65',lw=.5)
    axs[0,0].set_ylim(bottom=0);axs[0,1].set_ylim(bottom=0)
    phases=['roll_pos_hold','pitch_pos_hold','roll_neg_hold','pitch_neg_hold']
    matrix=np.array([[np.sqrt(np.mean(d.loc[d.switch_phase==p,'attitude_error_angle_deg']**2)) for p in phases] for d in data])
    ax=axs[1,1];norm=Normalize(0,1.2);cmap=plt.get_cmap('YlGnBu')
    im=matplotlib.cm.ScalarMappable(norm=norm,cmap=cmap)
    for (i,j),v in np.ndenumerate(matrix):
        ax.add_patch(plt.Rectangle((j-.5,i-.5),1,1,facecolor=cmap(norm(v)),edgecolor='white',lw=.5))
        ax.text(j,i,f'{v:.2f}',ha='center',va='center',fontsize=10,color='white' if v>.7 else '#152B38')
    ax.set_xlim(-.5,3.5);ax.set_ylim(1.5,-.5)
    ax.set(xticks=range(4),xticklabels=['+80 R','+180 P','-80 R','-180 P'],yticks=[0,1],yticklabels=['Isolation','Repeat'])
    ax.grid(False);fig.colorbar(im,ax=ax,shrink=.8,pad=.04,label='SO(3) RMS [deg]')
    for ax,letter,title in zip(axs.flat,'abcd',['Residual attitude transients','Position hold','True altitude','Peak-hold accuracy']):panel(ax,letter,title)
    fig.legend([Line2D([],[],color=c) for c in colors],['Both fixes: isolation run','Formal entry: repeat'],
        loc='upper center',bbox_to_anchor=(.53,.867),ncol=2,frameon=False)
    return fig


def mechanism():
    data=fix_frames();fig,axs=frame('Why the old allocation model misled the optimizer',
        'Same measured-state correction; only servo equilibrium gain changes after sensor repair',
        'Wrench traces are model estimates, NOT independently measured torque. No hardware gain change.\nShading marks the first +80 deg hold. Curves show all recorded samples.',shape=(2,2),height=5.8)
    for j,n in enumerate([1,2]):
        s=data[n][data[n].motion_time.between(8,20)]
        q=s.switch_measured_joint_b_l_rad;u=s.switch_commanded_joint_b_l_rad
        k=np.where(u>=0,.9018,.9119) if j==0 else np.ones(len(u))
        ax=axs[0,j]
        ax.plot(s.motion_time,np.degrees(u),color='#282C30',ls='--',label='Command u')
        ax.plot(s.motion_time,np.degrees(q),color='#007F73',label='Measured q')
        ax.plot(s.motion_time,np.degrees(k*u),color='#D58A00',ls=':',label='Equilibrium Ku')
        ax.set(ylabel='Left second-stage joint [deg]',xlabel='Time from first roll [s]')
        ax.set_ylim(-85,65)
        detail=ax.inset_axes([.56,.53,.41,.40])
        detail.plot(s.motion_time,np.degrees(u),color='#282C30',ls='--',lw=.8)
        detail.plot(s.motion_time,np.degrees(q),color='#007F73',lw=.8)
        detail.plot(s.motion_time,np.degrees(k*u),color='#D58A00',ls=':',lw=.8)
        detail.set(xlim=(14,20),ylim=(-79,-59),xticks=[14,20],yticks=[-75,-65])
        detail.tick_params(labelsize=5,pad=1);detail.set_title('Hold detail',fontsize=6,pad=2)
        ax=axs[1,j]
        for col,c,ls,label in [('wrench_tau_z_nm','#282C30','--','Requested'),
            ('drcda_estimated_wrench_tz_nm','#007F73','-','Current estimate'),
            ('drcda_predicted_wrench_tz_nm','#D58A00',':','Terminal prediction')]:
            ax.plot(s.motion_time,s[col],color=c,ls=ls,lw=.9,label=label)
        ax.set(ylabel='Yaw torque [Nm]',xlabel='Time from first roll [s]')
        ax.set_ylim(-2.1,1.1)
        for ax in axs[:,j]:ax.axvspan(15,20,color='#CDD1D6',alpha=.18,zorder=0);ax.set_xlim(8,20)
    for ax,letter,title in zip(axs.flat,'abcd',['Sensor fixed; old DC gain','Sensor fixed; unit DC gain','Old model: predicted correction','Corrected model: consistency']):panel(ax,letter,title)
    axs[0,0].legend(loc='lower left',fontsize=6.4,framealpha=.9)
    axs[1,1].legend(loc='lower left',fontsize=6.4,framealpha=.9)
    fig.text(.055,.856,'A steady joint should not be predicted to move away from an already reached command.',fontsize=8,color='#50575D')
    return fig


FIGURES=[('01_lateral_comparison',lateral),('02_lissajous_3d',lissajous),
    ('03_lissajous_ablation',ablation),('04_aggressive_stress',stress),
    ('05_first_roll_diagnosis',diagnosis),('06_large_attitude',full_attitude),
    ('07_allocator_mechanism',mechanism)]


def main():
    global OUT
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=Path(__file__).resolve().parent/'publication_figures_20261012')
    a=p.parse_args();OUT=a.output;OUT.mkdir(parents=True,exist_ok=True)
    for name in ['TRO','SR']:
        style(name);dest=OUT/name;dest.mkdir(exist_ok=True)
        with PdfPages(OUT/f'{name}_comparison_atlas.pdf') as atlas:
            atlas.infodict().update(Title='Hnuter no-delay SITL comparison and diagnostic figures',Author='Hnuter experiment archive',Subject='Historical comparisons separated from post-fix diagnostic evidence')
            for stem,func in FIGURES:
                fig=func()
                fig.savefig(dest/f'{stem}.pdf')
                fig.savefig(dest/f'{stem}.svg')
                fig.savefig(dest/f'{stem}.png',dpi=450)
                atlas.savefig(fig)
                plt.close(fig)
                print(name,stem,flush=True)
    (OUT/'sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
    from PIL import Image,ImageOps,ImageDraw
    thumbnails=[]
    for stem,_ in FIGURES:
        img=Image.open(OUT/'SR'/f'{stem}.png').convert('RGB')
        img.thumbnail((700,590));tile=Image.new('RGB',(730,620),'#E9EDF0')
        tile.paste(img,((730-img.width)//2,10));ImageDraw.Draw(tile).text((15,602),stem,fill='#24313B')
        thumbnails.append(tile)
    sheet=Image.new('RGB',(1460,2480),'#E9EDF0')
    for i,img in enumerate(thumbnails):sheet.paste(img,((i%2)*730,(i//2)*620))
    sheet.save(OUT/'overview.png')
    if Path(__file__).resolve()!=(OUT/'build_publication_figures.py').resolve():
        shutil.copy2(__file__,OUT/'build_publication_figures.py')
    print(OUT.resolve())


if __name__=='__main__':main()
