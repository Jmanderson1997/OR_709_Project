CART_MASS = 1   # mass of cart
POLE_MASS = .1  # mass of pole
POLE_LENGTH = .5    # length of pole
TIME_DIFF = .02  # Time differential between state observations
FORCE_MAGNITUDE = 10    # magnitude of force applied to cart
GRAVITY_ACC = 9.8   # acceleration due to gravity
import math


# reference https://coneural.org/florian/papers/05_cart_pole.pdf

# This is only for readability. Easy to just use the array for all of this
def create_dict(state_vec): 
    assert len(state_vec) == 4, "invalid state vector"
    return {"cart_pos": state_vec[0], "cart_vel": state_vec[1], "pole_angle": state_vec[2], "ang_vel":state_vec[3]}


def update_state(state, choice): 
    assert "cart_pos" in  state, "State missing cart's position"
    assert "cart_vel" in  state, "State missing cart's velocity"
    assert "pole_angle" in  state, "State missing pole's angle"
    assert "ang_vel" in  state, "State missing angular velocity"

    force = FORCE_MAGNITUDE if choice else -FORCE_MAGNITUDE
    angular_acc = angular_acceleration(state, force)
    cart_acc = cart_acceleration(state, force, angular_acc)

    state["cart_pos"] += TIME_DIFF * state["cart_vel"]
    state["cart_vel"] += TIME_DIFF * cart_acc
    state["pole_angle"] += TIME_DIFF * state["ang_vel"]
    state["ang_vel"] += TIME_DIFF * angular_acc
    return state


def angular_acceleration(state, force):
    theta = state["pole_angle"]
    theta_prime = state["ang_vel"]
    total_mass = CART_MASS + POLE_MASS

    numerator = -force - (POLE_MASS * POLE_LENGTH * theta_prime**2 *  math.sin(theta))
    numerator /= total_mass
    numerator = GRAVITY_ACC * math.sin(theta) + numerator * math.cos(theta)

    denominator = (POLE_MASS * math.cos(theta)**2) / total_mass
    denominator = 4.0/3.0 - denominator
    denominator *= POLE_LENGTH
    
    return numerator/denominator

def cart_acceleration(state, force, ang_acc): 
    theta = state["pole_angle"]
    theta_prime = state["ang_vel"]
    total_mass = CART_MASS + POLE_MASS

    numerator = force + POLE_MASS*POLE_LENGTH*(theta_prime**2 * math.sin(theta) - ang_acc*math.cos(theta))
    return numerator/total_mass

