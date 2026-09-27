"""Dependency-free planar four-wheel steering kinematics (SI units)."""
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def config():
    path = ROOT / 'config/robot.json'
    if not path.exists():
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('sim_car')) / 'config/robot.json'
    return json.loads(path.read_text())

def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def approach(a, b, step):
    return a + max(-step, min(step, b-a))

class Simulator:
    def __init__(self, cfg=None):
        self.cfg = cfg or config()
        a, b = self.cfg['wheelbase']/2, self.cfg['track']/2
        self.positions = [(a,b), (a,-b), (-a,b), (-a,-b)]
        self.names = ['fl','fr','rl','rr']
        self.angles = [0.0]*4
        self.spins = [0.0]*4
        self.speeds = [0.0]*4
        self.pose = [0.0]*3
        self.velocity = [0.0]*3
        self.time = 0.0
        self.aligning = False

    def inverse(self, command):
        result = []
        for (x,y), current in zip(self.positions, self.angles):
            u,v = command[0]-command[2]*y, command[1]+command[2]*x
            speed = math.hypot(u,v)
            angle = math.atan2(v,u) if speed > 1e-9 else current
            delta = wrap(angle-current)
            if abs(delta) > math.pi/2:
                delta = wrap(delta+math.pi)
                speed = -speed
            result.append((current+delta,speed))
        return result

    def step(self, command, dt):
        if dt <= 0: raise ValueError('dt must be positive')
        c = self.cfg
        target = list(command)
        mag = math.hypot(*target[:2])
        if mag > c['max_wheel_speed']:
            target[:2] = [v*c['max_wheel_speed']/mag for v in target[:2]]
        target[2] = max(-c['command_angular_limit'], min(c['command_angular_limit'], target[2]))
        peak = max(abs(s) for _,s in self.inverse(target))
        if peak > c['max_wheel_speed']:
            target = [v*c['max_wheel_speed']/peak for v in target]
        goals = self.inverse(target)
        error = max(abs(a-b) for (a,_),b in zip(goals,self.angles))
        # Brake along the current twist before changing steering direction.
        # This conservative stop-steer-drive policy maintains rolling compatibility.
        if error > 1e-6:
            self.aligning = True
            speed = math.hypot(*self.velocity[:2])
            omega = abs(self.velocity[2])
            fraction = max(speed/(c['linear_acceleration']*dt), omega/(c['angular_acceleration']*dt), 1)
            factor = max(0, 1-1/fraction)
            self.velocity = [v*factor for v in self.velocity]
            if max(abs(v) for v in self.velocity) < 1e-9:
                self.velocity = [0.0]*3
                self.angles = [approach(old,a,c['steer_rate']*dt) for old,(a,_) in zip(self.angles,goals)]
        else:
            self.aligning = False
            delta = [a-b for a,b in zip(target,self.velocity)]
            fraction = max(math.hypot(*delta[:2])/(c['linear_acceleration']*dt), abs(delta[2])/(c['angular_acceleration']*dt), 1)
            self.velocity = [v+d/fraction for v,d in zip(self.velocity,delta)]
        vx,vy,w = self.velocity
        self.speeds = [(vx-w*y)*math.cos(a)+(vy+w*x)*math.sin(a) for (x,y),a in zip(self.positions,self.angles)]
        self.spins = [p+s/c['wheel_radius']*dt for p,s in zip(self.spins,self.speeds)]
        theta = self.pose[2]
        if abs(w) < 1e-10:
            dx,dy = vx*dt,vy*dt
        else:
            sn,co = math.sin(w*dt),math.cos(w*dt)
            dx,dy = (sn*vx-(1-co)*vy)/w, ((1-co)*vx+sn*vy)/w
        self.pose[0] += math.cos(theta)*dx-math.sin(theta)*dy
        self.pose[1] += math.sin(theta)*dx+math.cos(theta)*dy
        self.pose[2] += w*dt
        self.time += dt
        return self.pose

def demo_command(t):
    stages = [(4,(.6,0,0)),(8,(0,.5,0)),(12,(.4,.4,0)),(17,(0,0,.7)),(24,(.5,0,.4)),(28,(-.5,0,0)),(32,(0,0,0))]
    return next((v for end,v in stages if t < end), (0,0,0))
