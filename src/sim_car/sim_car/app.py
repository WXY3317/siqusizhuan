import argparse
import csv
import math
from pathlib import Path
from .core import Simulator, demo_command

def export(directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory=Path(directory); directory.mkdir(parents=True,exist_ok=True)
    sim=Simulator(); rows=[]
    for _ in range(1600):
        command=demo_command(sim.time); sim.step(command,.02)
        rows.append([sim.time,*sim.pose,*sim.velocity,*sim.angles,*sim.speeds])
    with (directory/'trajectory.csv').open('w') as f:
        writer=csv.writer(f); writer.writerow(['time','x','y','yaw','vx','vy','omega',*[n+'_steer_rad' for n in sim.names],*[n+'_speed_mps' for n in sim.names]]); writer.writerows(rows)
    fig,axs=plt.subplots(2,2,figsize=(12,8)); t=[r[0] for r in rows]
    axs[0,0].plot([r[1] for r in rows],[r[2] for r in rows]); axs[0,0].set(xlabel='x (m)',ylabel='y (m)',title='Poseidon planar trajectory'); axs[0,0].axis('equal')
    for i,n in enumerate(['vx','vy','omega']): axs[0,1].plot(t,[r[4+i] for r in rows],label=n)
    axs[0,1].set(title='Body velocity (m/s, rad/s)')
    for i,n in enumerate(sim.names):
        axs[1,0].plot(t,[math.degrees(r[7+i]) for r in rows],label=n)
        axs[1,1].plot(t,[r[11+i] for r in rows],label=n)
    axs[1,0].set(title='Steering angle (deg)',xlabel='Time (s)'); axs[1,1].set(title='Wheel rolling speed (m/s)',xlabel='Time (s)')
    for ax in axs.flat:
        ax.grid(True,alpha=.3)
        if ax!=axs[0,0]: ax.legend()
    fig.tight_layout(); fig.savefig(directory/'simulation.png',dpi=150); plt.close(fig)
    print(f'Exported {directory}/simulation.png and trajectory.csv')

def gui():
    import tkinter as tk
    sim=Simulator(); root=tk.Tk(); root.title('Poseidon 四驱四转运动学仿真'); root.geometry('1100x800')
    canvas=tk.Canvas(root,bg='#101c2c',height=590); canvas.pack(fill='both',expand=True)
    panel=tk.Frame(root); panel.pack(fill='x'); sliders=[]
    for name,lo,hi in [('vx (m/s)',-1.5,1.5),('vy (m/s)',-1.5,1.5),('ω (rad/s)',-1,1)]:
        s=tk.Scale(panel,label=name,from_=lo,to=hi,resolution=.05,orient='horizontal',length=230); s.pack(side='left'); sliders.append(s)
    automatic=tk.BooleanVar(value=True); tk.Checkbutton(panel,text='自动演示',variable=automatic).pack(side='left')
    paused=tk.BooleanVar(); tk.Checkbutton(panel,text='暂停',variable=paused).pack(side='left')
    trail=[]
    def reset():
        nonlocal sim
        sim=Simulator(); trail.clear()
    tk.Button(panel,text='重置',command=reset).pack(side='left')
    def stop():
        automatic.set(False)
        for s in sliders:s.set(0)
    tk.Button(panel,text='停止',command=stop).pack(side='left')
    info=tk.Label(root,justify='left',font=('monospace',11)); info.pack(fill='x')
    tk.Label(root,text='平地无滑移 · 转向时先停车 · 轮径/转向速度为假设 · 轨迹随车居中显示',fg='#555555').pack()
    def tick():
        if not paused.get():
            sim.step(demo_command(sim.time%32) if automatic.get() else [s.get() for s in sliders],.02)
            trail.append(tuple(sim.pose[:2])); del trail[:-6000]
        canvas.delete('all'); w,h=canvas.winfo_width(),canvas.winfo_height(); scale=140
        def point(x,y):return (w/2+(x-sim.pose[0])*scale,h/2-(y-sim.pose[1])*scale)
        for axis in range(2):
            center=sim.pose[axis]
            for i in range(math.floor(center)-8,math.floor(center)+9):
                xy=(point(i,sim.pose[1]-8)+point(i,sim.pose[1]+8)) if axis==0 else (point(sim.pose[0]-8,i)+point(sim.pose[0]+8,i))
                canvas.create_line(*xy,fill='#23354a')
        if len(trail)>1:canvas.create_line(*[v for p in trail for v in point(*p)],fill='#41d9c6',width=2)
        def local(x,y):
            a=sim.pose[2]; return point(sim.pose[0]+x*math.cos(a)-y*math.sin(a),sim.pose[1]+x*math.sin(a)+y*math.cos(a))
        l,b=sim.cfg['length']/2,sim.cfg['width']/2
        canvas.create_polygon(*[v for p in [(l,b),(l,-b),(-l,-b),(-l,b)] for v in local(*p)],fill='#295478',outline='#91b8d0',width=2)
        canvas.create_line(*local(0,0),*local(.23,0),arrow='last',fill='white',width=3)
        for n,(x,y),a in zip(sim.names,sim.positions,sim.angles):
            dx,dy=.07*math.cos(a),.07*math.sin(a)
            canvas.create_line(*local(x-dx,y-dy),*local(x+dx,y+dy),fill='#ffce66',width=10)
            canvas.create_text(*local(x,y+.10),text=n,fill='white')
        info.config(text=f't={sim.time:6.2f}s  x={sim.pose[0]:+.3f}m  y={sim.pose[1]:+.3f}m  yaw={math.degrees(sim.pose[2]):+.1f}°  状态：'+('制动/转向' if sim.aligning else '行驶/停止')+'\n'+'  '.join(f'{n}: {math.degrees(a):+.1f}° / {s:+.2f}m/s' for n,a,s in zip(sim.names,sim.angles,sim.speeds)))
        root.after(20,tick)
    tick(); root.mainloop()

def main():
    p=argparse.ArgumentParser(); p.add_argument('--export',metavar='DIRECTORY'); a=p.parse_args()
    export(a.export) if a.export else gui()
if __name__=='__main__':main()
