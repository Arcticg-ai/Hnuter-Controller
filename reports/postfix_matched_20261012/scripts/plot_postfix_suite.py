"""Journal-style comparisons using only the matched post-fix experiment matrix."""
import argparse
from functools import lru_cache
import json
import os
from pathlib import Path
import shutil

os.environ.setdefault('MPLCONFIGDIR','/tmp/hnuter-postfix-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.colors import LogNorm
from matplotlib.lines import Line2D
import numpy as np
import pandas as pd
from PIL import Image,ImageDraw
from build_publication_figures import style,frame,panel,legend,METHODS,LABELS,COLORS,STYLES

ROOT=Path('/home/hnuter/px4_ws_ros2_consolidated/reports/postfix_matched_20261012')
METRICS=None
PHASES=None
MAIN=['direct','paper_nda','full']
ABLATION=['full','no_preview','no_horizon','no_nearfield']
FOOT='Corrected magnetic frames and unit servo DC gain; no-delay SITL; direct motor/servo commands.\nIdentical outer-loop gains and limits. One run per condition; no confidence intervals or smoothing.'
TIME_FOOT=FOOT+'\nGray bands: peak holds (attitude task) or post-maneuver settling (Lissajous).'


@lru_cache(None)
def data(s,m):
    return pd.read_csv(ROOT/'data'/s/'repeat_1'/m/'plot_data.csv',low_memory=False)


def row(s,m):return METRICS[(METRICS.scenario==s)&(METRICS.method==m)].iloc[0]


def line(ax,d,column,m):
    ax.plot(d.t,d[column],color=COLORS[m],ls=STYLES[m],lw=1.05,label=LABELS[m])


def time_axes(ax,label,scenario):
    end=165 if scenario.startswith('attitude') else (12.75 if scenario=='lissajous_7' else 10.75)
    limit=max(end,float(METRICS.loc[METRICS.scenario==scenario,'evaluated_duration_s'].max()))
    ax.set(xlabel='Time from motion start [s]',ylabel=label,xlim=(0,limit))
    if scenario.startswith('attitude'):
        for t in [0,40,80,120]:
            ax.axvline(t,color='.65',lw=.5)
            ax.axvspan(t+15,t+20,color='#CBD2D5',alpha=.2,zorder=0)
    else:ax.axvspan(end-5,limit,color='#CBD2D5',alpha=.2,zorder=0)


def spatial():
    s='lissajous_7';fig,axs=frame('3D Lissajous: matched method comparison',
        'T = 7 s + 0.75 s shared phase ramp | black dotted curves: each run reference',TIME_FOOT,height=6.0)
    spec=axs[0,0].get_subplotspec();axs[0,0].remove();a3=fig.add_subplot(spec,projection='3d')
    xyz=['position_x_enu_m','position_y_enu_m','position_z_rel_m']
    refs=['target_x_enu_m','target_y_enu_m','target_z_rel_m']
    for m in MAIN:
        d=data(s,m);moved=d[d.t<=7.75]
        origin=d[refs].iloc[0].to_numpy();origin[2]=0
        actual=moved[xyz].values-origin;desired=moved[refs].values-origin
        a3.plot(*actual.T,color=COLORS[m],ls=STYLES[m],lw=1.1)
        a3.plot(*desired.T,color='.25',ls=':',lw=.6,alpha=.7)
        axs[0,1].plot(actual[:,0],actual[:,1],color=COLORS[m],ls=STYLES[m])
        axs[0,1].plot(desired[:,0],desired[:,1],color='.25',ls=':',lw=.6,alpha=.7)
        line(axs[1,0],d,'ep',m);line(axs[1,1],d,'attitude_error_angle_deg',m)
    a3.set(xlabel='x [m]',ylabel='y [m]',zlabel='z [m]');a3.view_init(27,-62)
    a3.set_box_aspect((2.4,1.8,.7));a3.tick_params(pad=0,labelsize=6)
    for a in [a3.xaxis,a3.yaxis,a3.zaxis]:a.labelpad=0
    axs[0,1].set(xlabel='Relative ENU x [m]',ylabel='Relative ENU y [m]')
    axs[0,1].set_aspect('equal',adjustable='datalim')
    for ax,y in [(axs[1,0],'Position error [m]'),(axs[1,1],'Control SO(3) error [deg]')]:
        time_axes(ax,y,s);ax.set_ylim(bottom=0)
    for ax,letter,title in zip([a3,axs[0,1],axs[1,0],axs[1,1]],'abcd',
        ['Spatial path','Horizontal projection','Position tracking','Attitude tracking']):panel(ax,letter,title)
    legend(fig,MAIN);return fig


def stress():
    s='lissajous_5';fig,axs=frame('Aggressive Lissajous: matched comparison',
        'T = 5 s + 0.75 s shared phase ramp | all samples retained, including loss of tracking',TIME_FOOT,height=5.9)
    fields=[('ep','Position error [m]','Position tracking'),
        ('attitude_error_angle_deg','Control SO(3) error [deg]','Attitude tracking'),
        ('truth_relative_height_m','Gazebo true relative height [m]','Physical height'),
        ('true_reference_so3_deg','True reference error [deg]','Independent attitude check')]
    for ax,(column,ylabel,title),letter in zip(axs.flat,fields,'abcd'):
        for m in MAIN:line(ax,data(s,m),column,m)
        time_axes(ax,ylabel,s);panel(ax,letter,title)
        if column!='truth_relative_height_m':ax.set_ylim(bottom=0)
    axs[1,0].axhline(.5,ls=':',color='.35',lw=.7)
    for m in MAIN:
        loss=row(s,m).first_physical_altitude_loss_s
        if pd.notna(loss):axs[1,0].scatter(loss,.5,marker='X',s=30,color=COLORS[m],zorder=4)
    legend(fig,MAIN);return fig


def ablation(s):
    duration=7 if s.endswith('_7') else 5
    fig,axs=frame(f'{duration} s Lissajous: allocation ablations',
        'Full / no future reference / one-step horizon / no near-term objective',TIME_FOOT,height=6.0)
    for m in ABLATION:
        d=data(s,m)
        line(axs[0,0],d,'ep',m);line(axs[0,1],d,'attitude_error_angle_deg',m)
        errors=np.sort(d.ep)
        axs[1,0].plot(errors,np.arange(1,len(errors)+1)/len(errors),color=COLORS[m],ls=STYLES[m])
    for ax,label in [(axs[0,0],'Position error [m]'),(axs[0,1],'Control SO(3) error [deg]')]:
        time_axes(ax,label,s);ax.set_ylim(bottom=0)
    axs[1,0].set(xlabel='Position error [m]',ylabel='Within-run empirical CDF',ylim=(0,1.02))
    ax=axs[1,1]
    for i,m in enumerate(ABLATION):
        r=row(s,m);values=[r.commanded_joint_total_variation_rad,r.measured_joint_total_variation_rad]
        ax.plot(values,[i,i],color=COLORS[m],lw=1.3)
        ax.scatter(values[0],i,color=COLORS[m],s=23)
        ax.scatter(values[1],i,edgecolor=COLORS[m],facecolor='white',s=23,marker='s')
    ax.set(yticks=range(4),yticklabels=[LABELS[m] for m in ABLATION],xlabel='Joint total variation [rad]',xscale='log')
    ax.invert_yaxis();ax.text(.98,.98,'Filled: command\nOpen: measured',transform=ax.transAxes,ha='right',va='top',fontsize=6.4)
    for ax,letter,title in zip(axs.flat,'abcd',['Position tracking','Attitude tracking','Sample error distribution','Actuator activity (not energy)']):panel(ax,letter,title)
    legend(fig,ABLATION);return fig


def attitude(methods,ablation=False):
    s='attitude_80_180';fig,axs=frame('Large-attitude '+('ablations' if ablation else 'method comparison'),
        '+80 roll / +180 pitch / -80 roll / -180 pitch | 15 s transition, 5 s peak hold',
        TIME_FOOT+'\nPanels e-f are labeled range expansions; full failure traces remain in panels a-d.',shape=(3,2),height=8.4)
    fields=[('attitude_error_angle_deg','Control SO(3) [deg]','Attitude tracking'),
        ('ep','Position error [m]','Position hold'),
        ('truth_relative_height_m','True relative height [m]','Physical height'),
        ('estimator_truth_so3_deg','EKF versus Gazebo truth [deg]','Estimator consistency'),
        ('attitude_error_angle_deg','Control SO(3) [deg]','Expanded range: 0-16 deg'),
        ('ep','Position error [m]','Expanded range: 0-0.25 m')]
    for ax,(column,ylabel,title),letter in zip(axs.flat,fields,'abcdef'):
        for m in methods:line(ax,data(s,m),column,m)
        time_axes(ax,ylabel,s);panel(ax,letter,title)
        if column!='truth_relative_height_m':ax.set_ylim(bottom=0)
    axs[2,0].set_ylim(0,16);axs[2,1].set_ylim(0,.25)
    legend(fig,methods);return fig


def holds():
    fig,axs=frame('Large-attitude peak-hold accuracy','Each cell is the RMS of one 5 s hold; no population statistics',
        FOOT+'\nHatched cells overlap or follow physical height loss and are not successful hovering evidence.',height=6.0)
    for ax in axs.flat:ax.remove()
    gs=fig.add_gridspec(1,2,left=.20,right=.94,bottom=.24,top=.75,wspace=.6)
    names=['roll_pos_hold','pitch_pos_hold','roll_neg_hold','pitch_neg_hold']
    for k,(field,title) in enumerate([('so3_rms_deg','Control SO(3) RMS'),('true_so3_rms_deg','True reference SO(3) RMS')]):
        ax=fig.add_subplot(gs[0,k]);norm=LogNorm(.1,100);cmap=plt.get_cmap('YlGnBu')
        for i,m in enumerate(METHODS):
            for j,phase in enumerate(names):
                found=PHASES[(PHASES.scenario=='attitude_80_180')&(PHASES.method==m)&(PHASES.phase==phase)]
                valid=len(found) and not bool(found.iloc[0].after_height_loss)
                value=float(found.iloc[0][field]) if len(found) else np.nan
                ax.add_patch(plt.Rectangle((j-.5,i-.5),1,1,facecolor=cmap(norm(max(value,.1))) if valid else '#E1E3E5',
                    edgecolor='white',lw=.7,hatch=None if valid else '///'))
                ax.text(j,i,f'{value:.2f}' if valid else 'N/A',ha='center',va='center',fontsize=7,
                    color='white' if valid and value>8 else '#1B2B35')
        ax.set(xlim=(-.5,3.5),ylim=(5.5,-.5),xticks=range(4),xticklabels=['+R','+P','-R','-P'],
            yticks=range(6),yticklabels=[LABELS[m] for m in METHODS] if k==0 else [])
        ax.grid(False);panel(ax,'ab'[k],title)
        sm=matplotlib.cm.ScalarMappable(norm=norm,cmap=cmap)
        fig.colorbar(sm,ax=ax,orientation='horizontal',fraction=.05,pad=.12,label='Degrees (log color scale)',ticks=[.1,1,10,100])
    return fig


def summary():
    fig=plt.figure(figsize=(7.2,8.1));gs=fig.add_gridspec(1,3,left=.25,right=.97,bottom=.16,top=.82,width_ratios=[1.15,1.15,.5],wspace=.3)
    axes=[fig.add_subplot(gs[0,i]) for i in range(3)]
    fig.text(.055,.96,'Matched results at a glance',fontsize=12,fontweight='bold')
    fig.text(.055,.923,'Single-run results, separated by task | no failure samples removed',fontsize=7.5,color='#50575D')
    fig.text(.055,.04,FOOT+'\nStatus: green = yes; red = no. Air = airborne completion; Acc. = original accuracy criteria.\nAirborne requires finished/armed, no cutoff, and true height above 0.5 m.',fontsize=6.3,color='#50575D')
    positions=[];labels=[]
    for block,s in enumerate(['lissajous_7','lissajous_5','attitude_80_180']):
        for i,m in enumerate(METHODS):
            y=block*7+i;positions.append(y);labels.append(LABELS[m]);r=row(s,m)
            for ax,key in zip(axes[:2],['position_rms_m','so3_peak_deg']):
                ax.scatter(r[key],y,color=COLORS[m],marker='o' if r.completed_airborne else 'X',s=25)
                ax.annotate(f'{r[key]:.3f}' if key=='position_rms_m' else f'{r[key]:.2f}',(r[key],y),
                    xytext=(5,0),textcoords='offset points',va='center',fontsize=6.1)
            axes[2].scatter([0,1],[y,y],c=['#007F73' if r.completed_airborne else '#AB4949','#007F73' if r.passed else '#AB4949'],s=22,marker='s')
        for ax in axes:
            ax.axhspan(block*7-.5,block*7+5.5,color='#ECEFF1',alpha=.35,zorder=0)
        axes[0].text(-.8,block*7-.85,['7 s Lissajous','5 s Lissajous','Large attitude'][block],
            transform=axes[0].get_yaxis_transform(),fontsize=8,fontweight='bold')
    for ax,title in zip(axes,['Position RMS [m]','Peak control SO(3) [deg]','Status']):
        ax.set_ylim(20.0,-1.5);ax.set_title(title,fontsize=8);ax.set_yticks(positions)
    axes[0].set_yticklabels(labels,fontsize=7)
    for ax in axes[1:]:ax.set_yticklabels([]);ax.tick_params(axis='y',length=0)
    for ax,key in zip(axes[:2],['position_rms_m','so3_peak_deg']):
        ax.set_xscale('log');ax.set_xlim(METRICS[key].min()*.6,METRICS[key].max()*4)
    axes[2].set(xticks=[0,1],xticklabels=['Air','Acc.'],xlim=(-.6,1.6));axes[2].grid(False)
    return fig


def main():
    global ROOT,METRICS,PHASES
    p=argparse.ArgumentParser();p.add_argument('--report',type=Path,default=ROOT);a=p.parse_args();ROOT=a.report
    METRICS=pd.read_csv(ROOT/'metrics.csv');PHASES=pd.read_csv(ROOT/'phase_metrics.csv')
    assert len(METRICS)==18 and json.loads((ROOT/'verification.json').read_text())['complete_matrix']
    out=ROOT/'publication_figures';out.mkdir(exist_ok=True)
    figures=[('01_lissajous_7_comparison',spatial),('02_lissajous_5_comparison',stress),
        ('03_lissajous_7_ablation',lambda:ablation('lissajous_7')),('04_lissajous_5_ablation',lambda:ablation('lissajous_5')),
        ('05_large_attitude_comparison',lambda:attitude(MAIN)),('06_large_attitude_ablation',lambda:attitude(ABLATION,True)),
        ('07_large_attitude_holds',holds),('08_summary',summary)]
    for theme in ['TRO','SR']:
        style(theme);dest=out/theme;dest.mkdir(exist_ok=True)
        with PdfPages(out/f'{theme}_postfix_comparison.pdf') as atlas:
            for name,func in figures:
                fig=func();fig.savefig(dest/f'{name}.pdf');fig.savefig(dest/f'{name}.svg')
                fig.savefig(dest/f'{name}.png',dpi=450);atlas.savefig(fig);plt.close(fig)
                print(theme,name,flush=True)
    sheet=Image.new('RGB',(1460,2600),'#EDF0F2')
    for i,(name,_) in enumerate(figures):
        img=Image.open(out/'SR'/f'{name}.png').convert('RGB');img.thumbnail((700,610))
        x,y=(i%2)*730,(i//2)*650;sheet.paste(img,(x+(730-img.width)//2,y+10))
        ImageDraw.Draw(sheet).text((x+12,y+630),name,fill='#24313B')
    sheet.save(out/'overview.png')
    scripts=ROOT/'scripts'
    for path in [Path(__file__),Path(__file__).with_name('build_publication_figures.py')]:
        if path.resolve()!=(scripts/path.name).resolve():shutil.copy2(path,scripts/path.name)


if __name__=='__main__':main()
