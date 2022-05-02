from os import stat
from turtle import right
import numpy as np
import math
from sys_dynamic_eq import update_state

class DPModel: 

    def __init__(self, initial_state={"cart_pos": 0, "cart_vel":0, "pole_angle":0, "ang_vel":0}, pos_weight=5, stability_weight=1) -> None:
        self.cart_bound = 2.4
        self.angle_bound =  12 * 2 * math.pi / 360
        self.pos_weight = pos_weight
        self.stability_weight = stability_weight
        self.possible_states = []
        self.initial_state = initial_state
        self.total_steps = 0
        self.tau = 0.02
        self.done = False

    def find_path(self, n_timesteps):
        if self.done: 
            print("session finished please restart")
            return False
        self.calculate_pasts(n_timesteps, self.initial_state, [], 0)
        min_state_diff = math.inf
        min_state_idx = -1
        for i, pottential in enumerate(self.possible_states):
            state_diff = self.calc_state_diff(self.initial_state, pottential["state"])
            if state_diff <= min_state_diff: 
                min_state_idx = i
                min_state_diff = state_diff
        chosen_path = self.possible_states[min_state_idx]
        path = [not choice for choice in chosen_path["path"]]
        return self.execute_path(path)

    def execute_path(self, path): 
        state = self.initial_state
        for choice in path: 
            state = update_state(state, choice)
            self.total_steps += 1
            if not self.check_bounds(state): 
                print("Path Failed. Total balance, steps:{}, time:{}".format(self.total_steps, self.tau*self.total_steps))
                self.done = True
                return False
        print("Still going")
        self.initial_state = state
        return True
        

    def calculate_pasts(self, steps_to_go, state, prev_decisions, total_cost):
        if steps_to_go > 0: 
            left_state =  update_state(state.copy(), 0)
            if self.check_bounds(left_state): 
                left_decisions = prev_decisions.copy()
                left_decisions.append(0)
                self.calculate_pasts(steps_to_go-1, left_state, left_decisions, total_cost+self.calc_cost(left_state))
            else: 
                pass
            right_state = update_state(state.copy(), 1)
            if self.check_bounds(right_state):
                right_decisions = prev_decisions.copy()
                right_decisions.append(1)
                self.calculate_pasts(steps_to_go-1, right_state, right_decisions, total_cost+self.calc_cost(right_state))
            else:
                pass
        else: 
            final = {"state": state, "path": prev_decisions, "total_cost":total_cost}
            self.possible_states.append(final)

    def calc_state_diff(self, state1, state2):
        cart_pos_diff = state1["cart_pos"] - state2["cart_pos"]
        cart_vel_diff = state1["cart_vel"] - state2["cart_vel"]
        pole_angle_diff = state1["pole_angle"] - state2["pole_angle"]
        ang_vel_diff = state1["ang_vel"] - state2["ang_vel"]
        return abs(cart_pos_diff) + abs(cart_vel_diff) + abs(pole_angle_diff) + abs(ang_vel_diff)

    def calc_cost(self, state):
        c = self.pos_weight*(abs(state['cart_pos'])/self.cart_bound) + self.stability_weight*abs(state['cart_vel'])
        c += self.pos_weight*(abs(state['pole_angle'])/self.cart_bound) + self.stability_weight*abs(state['ang_vel'])
        return c

    def check_bounds(self, state): 
        pos = abs(state["cart_pos"]) <= self.cart_bound
        angle = abs(state["pole_angle"]) <= self.angle_bound
        return pos and angle
