from copy import copy
import numpy as np
import sys
from ruckig import InputParameter, OutputParameter, Result, Ruckig, Synchronization, ControlInterface, DurationDiscretization
# from ruckig import Reflexxes

def walk_through_trajectory(otg, inp):
    out_list = []
    out_pos =[]
    out_time= []
    out = OutputParameter(inp.degrees_of_freedom)

    res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)

        inp.current_position = out.new_position
        inp.current_velocity = out.new_velocity
        inp.current_acceleration = out.new_acceleration

        out_list.append(copy(out))
        out_pos.append((out.new_position))
        out_time.append(out.calculation_duration)

    return out_list,out_pos,out_time

def ruckig_pos(c_pos,t_pos,max_vel,max_acc,max_jerk,step_time):

    inp = InputParameter(6)
    c_pos = list(map(float, c_pos.split()))
    inp.current_position = c_pos
    inp.current_velocity = [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
    inp.current_acceleration = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    t_pos = list(map(float, t_pos.split()))
    inp.target_position = t_pos
    inp.target_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    inp.target_acceleration = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    max_vel = list(map(float, max_vel.split()))
    inp.max_velocity = max_vel
    max_acc = list(map(float, max_acc.split()))
    inp.max_acceleration = max_acc
    max_jerk = list(map(float, max_jerk.split()))
    inp.max_jerk = max_jerk

    # inp.minimum_duration = 5.0
    step_time = float(step_time)
    # otg = Reflexxes(inp.degrees_of_freedom, 0.005)
    otg = Ruckig(inp.degrees_of_freedom, step_time)

    out_list,out_pos,out_time = walk_through_trajectory(otg, inp)
    return out_pos

if __name__ == "__main__":
    c_pos_m = sys.argv[1]
    t_pos_m = sys.argv[2]
    max_vel_m = sys.argv[3]
    max_acc_m = sys.argv[4]
    max_jerk_m = sys.argv[5]
    step_time_m = sys.argv[6]
    result = ruckig_pos(c_pos_m, t_pos_m, max_vel_m, max_acc_m, max_jerk_m, step_time_m)
    print(result)


    