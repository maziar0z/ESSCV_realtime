"""ESSCV MPC for the road test (no preceding vehicle).

Port of the simulation's optimization.py (MPC-IDM repo, ESSCV-SUMO): same energy objective and the same
SPaT speed-window constraints, solved with SLSQP over a 10 s horizon in 0.5 s steps. Differences from the
simulation, all because of the road setting:
  * the signal is described by a SignalTimeline built from SPaT (current phase + time to its change, then
    typical durations) instead of a fixed cycle `step % t_cycle`;
  * free-flow speed is the approach's speed limit (ff_spd from the OBU message), not a constant;
  * no leader: no min/max spacing constraints. Far from the signal and after it, the car cruises toward
    the speed limit (the simulation's no-leader cruise constraint); within `border` metres of the stop bar
    the SPaT speed window applies.
Positions use the simulation's convention: x grows along the road and the stop bar is at D_SIG, so
x = D_SIG - pos_ego where pos_ego is the OBU distance to the stop bar.
"""
import math
import numpy as np
from scipy.optimize import minimize

D_SIG = 500.0          # stop bar location in the MPC's own coordinate (any constant)
DT = 0.5               # MPC step (s)
HORIZON = 20           # 10 s
MAX_ACC = 2.0          # m/s^2 (simulation Veh_Parameter().max_acc)
MAX_DEC = -4.5         # m/s^2
SAFE_RANGE = 1.0       # arrive no earlier than green start + 1 s (and no later than green end - 1 s)
STOP_DEC = 3.0         # m/s^2: red/yellow stop-bar constraint only if the car can stop at this deceleration
RED_MARGIN = 1.0       # m: stop this far before the stop bar
EPS = 1e-5

# typical phase lengths for phases after the current one (from the 70 logged road-test runs);
# only used when the car cannot make the current green
TYPICAL_YELLOW = 5.0
TYPICAL_RED = 40.0
TYPICAL_GREEN = 10.0   # short on purpose: aim for the start of the next green, not its end


class SignalTimeline:
    """Phases ahead of the car, built from the current SPaT state and its time to change.

    status: "green" / "yellow" / "red"; sec_to_change: SPaT minEndTime of the current phase (s), as used
    by RLRWS. Yellow counts as red for planning (as in the simulation).
    """

    ORDER = {"green": ("yellow", TYPICAL_YELLOW), "yellow": ("red", TYPICAL_RED), "red": ("green", TYPICAL_GREEN)}

    def __init__(self, status, sec_to_change, horizon_s=300.0):
        self.phases = []   # (start, end, status) relative to now
        t, state, end = 0.0, status, max(0.0, float(sec_to_change))
        while t < horizon_s:
            self.phases.append((t, end, state))
            state, dur = self.ORDER[state]
            t, end = end, end + dur

    def is_green(self, t):
        return self._phase(t)[2] == "green"

    def status(self, t):
        return self._phase(t)[2]

    def _phase(self, t):
        for p in self.phases:
            if p[0] <= t < p[1]:
                return p
        return self.phases[-1]

    def green_end(self, t):
        """End of the green phase containing t."""
        return self._phase(t)[1]

    def next_green(self, t):
        """(start, end) of the first green phase starting after t."""
        for p in self.phases:
            if p[2] == "green" and p[0] > t:
                return p[0], p[1]
        return self.phases[-1][0], self.phases[-1][1]


class Optimizer:
    def __init__(self, freev):
        self.n = HORIZON
        self.horizon = HORIZON
        self.dt = DT
        self.freev = freev
        self.max_acc = MAX_ACC
        self.max_dec = MAX_DEC
        self.d_sig = D_SIG
        self.small_val = EPS

    # objective function (identical to the simulation's fixed optimization.py)
    def objective(self, Acc_eco, Speed_eco, prev_acc):
        fuel = 0.0
        m1 = 1550
        rhoD = 1.2
        CdA = 0.8                         # Cd * A = 0.8
        CRR = 0.01
        Km = 1.1
        g = 9.8
        V_eco = [self.freev-5] * (self.horizon+1)
        w = 18000 #weight for comf
        w2 = 1500    # weight for acc change
        w3 = 200
        V_eco [0] = Speed_eco
        for i in range (0, self.n):
            if CRR*m1*g * V_eco[i] + Km*m1*Acc_eco[i]*V_eco[i] + 0.5* rhoD * CdA * V_eco[i]**3 + w*Acc_eco[i]**2 <= 0:
                fuel += 0
            else:
                if i>0:
                    fuel += CRR*m1*g * V_eco[i] + Km*m1*Acc_eco[i]*V_eco[i] + 0.5* rhoD * CdA * V_eco[i]**3 +  w*Acc_eco[i]**2 + w2* (Acc_eco[i] -  Acc_eco[i-1])**2
                else:
                    fuel += CRR*m1*g * V_eco[i] + Km*m1*Acc_eco[i]*V_eco[i] + 0.5* rhoD * CdA * V_eco[i]**3 +  w*Acc_eco[i]**2 + w3 * (Acc_eco[i] -  prev_acc)**2
            V_eco[i+1] = V_eco[i] + Acc_eco[i]*self.dt
        return fuel

    def constraint0(self, Acc_eco, Posi_eco, Speed_eco, timeline):
        din2 = 10
        border = din2 * self.freev   # SPaT window applies within this distance of the stop bar
        V_eco = [0] * (self.horizon+1)
        V_eco[0] = Speed_eco
        positioneco = [0] * (self.horizon+1)
        positioneco[0] = Posi_eco
        consspd = [0] * self.horizon     # min speed
        consspd3 = [0] * self.horizon    # max speed
        consspd5 = [0] * self.horizon    # cruise lower bound (no leader)
        conssignal1 = [0] * self.horizon
        conssignal2 = [0] * self.horizon
        consacc1 = [0] * self.horizon
        consacc2 = [0] * self.horizon
        consred = []   # stay before the stop bar while the light is not green (added for the road test)
        # In SUMO the simulator itself stopped cars at red; on the road the MPC must. Enforced only if the car
        # can still stop comfortably (otherwise, e.g. at yellow onset in the dilemma zone, it goes through).
        dist0 = self.d_sig - Posi_eco
        enforce_red = must_stop(dist0, Speed_eco, self.freev, timeline)
        stop_limit = max(self.d_sig - RED_MARGIN, Posi_eco)   # already inside the margin: do not move further

        for i in range(0, self.horizon):
            positioneco[i+1] = positioneco[i] + V_eco[i]*self.dt + 0.5 * Acc_eco[i] * self.dt**2
            V_eco[i+1] = V_eco[i] + Acc_eco[i]*self.dt
            t = i*self.dt
            dist = self.d_sig - positioneco[i]

            if timeline is not None and timeline.is_green(t):   # green now (at step i)
                time_this = timeline.green_end(t) - t   # time left in this green
                spat_spd = dist / (time_this + self.small_val)
                if time_this < 1 and dist > 40: # last second of green light
                    spat_spd = min(spat_spd, self.freev*.6)
                elif time_this < 1 and dist <= 40:
                    spat_spd = min(spat_spd, 0.1)
                spat_spd = max(spat_spd, 3)
                spat_spd_low = spat_spd
                spat_spd_up = self.freev
                if spat_spd > self.freev + 0.1: # cannot make it: wait for the next green
                    g_start, g_end = timeline.next_green(t)
                    spat_spd_low = dist / (g_end - SAFE_RANGE - t)   # arrive before that green ends
                    spat_spd_up = dist / (g_start + SAFE_RANGE - t)  # and not before it starts (+1 s)
            elif timeline is not None:   # red or yellow now
                g_start, g_end = timeline.next_green(t)
                spat_spd_low = dist / (g_end - SAFE_RANGE - t)
                spat_spd_up = dist / (g_start + SAFE_RANGE - t)
                if spat_spd_up > self.freev:
                    spat_spd_up = self.freev
                    if spat_spd_low > self.freev:
                        spat_spd_low = V_eco[i] * 0.9
            else:   # no SPaT: no window
                spat_spd_low, spat_spd_up = 0.0, self.freev

            spat_spd_up = max(V_eco[i], spat_spd_up)

            consspd[i] = V_eco[i+1] - (-0.01)
            consspd3[i] = (self.freev + 0.1) - V_eco[i+1]
            consspd5[i] = V_eco[i+1] - min(self.freev-2, Speed_eco + 0.7*self.max_acc*self.dt*(i+1))
            conssignal1[i] = V_eco[i+1] - spat_spd_low
            conssignal2[i] = spat_spd_up - V_eco[i+1]
            consacc1[i] = self.max_acc - Acc_eco[i]
            consacc2[i] = Acc_eco[i] - self.max_dec
            if enforce_red and not timeline.is_green((i + 1) * self.dt):
                consred.append(stop_limit - positioneco[i+1])

        if timeline is not None and (self.d_sig - border) <= Posi_eco <= self.d_sig:   # approaching the signal
            return np.concatenate((conssignal1, conssignal2, consspd, consspd3, consacc1, consacc2, consred))
        if consred:   # far from the signal but already inside the red-light stopping range
            return np.concatenate((consspd, consspd3, consacc1, consacc2, consred))
        # far from the signal, after it, or no SPaT: cruise toward the speed limit
        return np.concatenate((consspd5, consspd, consspd3, consacc1, consacc2))


def must_stop(pos_ego, spd_ego, freev, timeline):
    """True when the car has to stop before the stop bar (the MPC's stop-bar constraint is active):
      red now                      -> always (before the bar)
      yellow now                   -> if it can still stop at STOP_DEC (dilemma zone: otherwise go through)
      green now but can't make it  -> if it can still stop at STOP_DEC (like RLRWS's tl_constraint)"""
    if timeline is None or pos_ego < 0:
        return False
    state = timeline.status(0)
    if state == "red":
        return True
    can_stop = pos_ego - RED_MARGIN >= spd_ego**2 / (2 * STOP_DEC) or spd_ego < 2.0
    if state == "green":
        return can_stop and pos_ego / max(timeline.green_end(0), EPS) > freev + 0.1
    return can_stop


def stop_speed(pos_ego, spd_ego, plan, t_ahead=1.5):
    """Speed to show when the car must stop: the MPC's planned speed ~t_ahead s ahead, never above the
    speed from which a STOP_DEC stop still ends at the bar (so the display can't say 'go' before a red bar
    even if a solve failed)."""
    return min(planned_speed(spd_ego, plan, t_ahead), math.sqrt(2 * STOP_DEC * max(pos_ego - RED_MARGIN, 0.0)))


def planned_speed(spd_ego, plan, t_ahead):
    """Speed the MPC plan reaches t_ahead seconds from now."""
    k = int(round(t_ahead / DT))
    return max(0.0, spd_ego + DT * float(np.sum(np.clip(plan[:k], MAX_DEC, MAX_ACC))))


def solve_mpc(pos_ego, spd_ego, freev, timeline, warm_start, prev_acc):
    """One MPC solve. pos_ego = distance to the stop bar (m, >0 before it). Returns (plan, result)."""
    opt = Optimizer(freev)
    posi_eco = D_SIG - pos_ego
    cons = {"type": "ineq", "fun": opt.constraint0, "args": (posi_eco, spd_ego, timeline)}
    sol = minimize(opt.objective, warm_start, args=(spd_ego, prev_acc), method="SLSQP", constraints=[cons])
    return np.asarray(sol.x, dtype=float), sol


def initial_guess():
    return np.full(HORIZON + 1, 0.05)


def shifted_warm_start(plan):
    """Next solve's start: last plan shifted by the 2 steps (1 s) since it was computed, clipped to bounds
    (a failed SLSQP solve can return huge values that would poison the next solve)."""
    nxt = np.append(plan[2:], [plan[-1], plan[-1]])
    return np.clip(nxt, MAX_DEC, MAX_ACC)
