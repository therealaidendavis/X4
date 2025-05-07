import numpy as np

def calc_state(P, T, R=287):
    V = R * T / P
    return P, T, V

class DualCycle:
    def __init__(self, r=18, rc=1.2, rp=1.5, T1=300, P1=1e5, gamma=1.4, R=287):
        self.r = r
        self.rc = rc
        self.rp = rp
        self.T1 = T1
        self.P1 = P1
        self.gamma = gamma
        self.R = R

    def solve(self):
        r = self.r
        rc = self.rc
        rp = self.rp
        gamma = self.gamma
        R = self.R

        T1 = self.T1
        P1 = self.P1
        V1 = R * T1 / P1

        V2 = V1 / r
        T2 = T1 * (r) ** (gamma - 1)
        P2 = P1 * (r) ** gamma

        V3 = V2
        P3 = rp * P2
        T3 = P3 * V3 / (R)

        V4 = rc * V3
        T4 = T3 * (rc)
        P4 = P3

        V5 = V1
        T5 = T4 * (V4 / V5) ** (1 - gamma)
        P5 = P4 * (V4 / V5) ** gamma

        cv = R / (gamma - 1)
        cp = gamma * cv

        Q23 = cv * (T3 - T2)
        Q34 = cp * (T4 - T3)
        Q41 = cv * (T5 - T1)

        Qin = Q23 + Q34
        Qout = Q41

        Wnet = Qin - Qout
        eta = 1 - (Qout / Qin)

        states = {
            "1": (P1, T1, V1),
            "2": (P2, T2, V2),
            "3": (P3, T3, V3),
            "4": (P4, T4, V4),
            "5": (P5, T5, V5),
        }

        results = {
            "states": states,
            "Qin": Qin,
            "Qout": Qout,
            "Wnet": Wnet,
            "eta": eta,
        }

        return results
