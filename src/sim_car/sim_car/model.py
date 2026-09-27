from pathlib import Path
from .core import config

def generate():
    c=config(); r=c['wheel_radius']; parts=['<?xml version="1.0"?><robot name="poseidon">', '<link name="base_footprint"/>', '<joint name="base_height" type="fixed"><parent link="base_footprint"/><child link="base_link"/><origin xyz="0 0 0.16"/></joint>']
    def link(name, geometry, color):
        return f'<link name="{name}"><visual><geometry>{geometry}</geometry><material name="{name}_mat"><color rgba="{color}"/></material></visual><collision><geometry>{geometry}</geometry></collision></link>'
    parts.append(link('base_link',f'<box size="{c["length"]} {c["width"]} 0.08"/>','0.12 0.28 0.40 1'))
    # Low battery box leaves 40 mm ground clearance; sensors are illustrative only.
    parts.append(link('battery','<box size="0.24 0.22 0.08"/>','0.2 0.2 0.23 1'))
    parts.append('<joint name="battery_mount" type="fixed"><parent link="base_link"/><child link="battery"/><origin xyz="0 0 -0.08"/></joint>')
    for name,x,y in [('fl',1,1),('fr',1,-1),('rl',-1,1),('rr',-1,-1)]:
        parts.append(link(name+'_steer','<box size="0.07 0.045 0.05"/>','0.75 0.55 0.15 1'))
        parts.append(f'<joint name="{name}_steer_joint" type="continuous"><parent link="base_link"/><child link="{name}_steer"/><origin xyz="{x*c["wheelbase"]/2} {y*c["track"]/2} {r-0.16}"/><axis xyz="0 0 1"/></joint>')
        wheel=link(name+'_wheel',f'<cylinder radius="{r}" length="{c["wheel_width"]}"/>','0.12 0.12 0.13 1')
        wheel=wheel.replace('<geometry>','<origin rpy="1.57079632679 0 0"/><geometry>')
        parts.append(wheel)
        parts.append(f'<joint name="{name}_wheel_joint" type="continuous"><parent link="{name}_steer"/><child link="{name}_wheel"/><axis xyz="0 1 0"/></joint>')
    for i,sign in enumerate([1,-1]):
        parts.append(link(f'sensor_{i}','<cylinder radius="0.035" length="0.06"/>','0.2 0.65 0.8 1'))
        parts.append(f'<joint name="sensor_mount_{i}" type="fixed"><parent link="base_link"/><child link="sensor_{i}"/><origin xyz="{sign*0.22} {sign*0.18} {c["height"]-0.03-0.16}"/></joint>')
    parts.append('</robot>')
    return '\n'.join(parts)

if __name__=='__main__':
    (Path(__file__).resolve().parents[1] / 'urdf/poseidon.urdf').write_text(generate())
