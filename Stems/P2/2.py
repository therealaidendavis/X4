import math
import random
import time
from statistics import mean, stdev

# ---------- Position Class ----------
class Position():
    def __init__(self, pos=None, x=None, y=None, z=None):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        if pos is not None:
            self.x, self.y, self.z = pos
        self.x = x if x is not None else self.x
        self.y = y if y is not None else self.y
        self.z = z if z is not None else self.z

    def __add__(self, other):
        return Position((self.x + other.x, self.y + other.y, self.z + other.z))

    def __iadd__(self, other):
        if type(other) == Position:
            self.x += other.x
            self.y += other.y
            self.z += other.z
        return self

    def __sub__(self, other):
        return Position((self.x - other.x, self.y - other.y, self.z - other.z))

    def __truediv__(self, other):
        return Position((self.x / other, self.y / other, self.z / other))

    def __mul__(self, other):
        if type(other) in (float, int):
            return Position((self.x * other, self.y * other, self.z * other))
        if isinstance(other, Position):
            return Position((self.x * other.x, self.y * other.y, self.z * other.z))

    def __rmul__(self, other):
        return self * other

    def mag(self):
        return (self.x ** 2 + self.y ** 2 + self.z ** 2) ** 0.5

    def normalize(self):
        l = self.mag()
        if l > 0:
            self.x /= l
            self.y /= l
            self.z /= l

    def distTo(self, p2=None):
        return (self - p2).mag() if p2 else self.mag()

    def getTup(self):
        return (self.x, self.y, self.z)

    def getRndDir(self):
        """
        Returns a random unit vector in 3D space.
        """
        random.seed(time.time())
        # Generate a random direction vector
        d = Position(x=random.random(), y=random.random(), z=random.random())
        # Shift by (0.5, 0.5, 0.5) to center around origin
        d = d - Position(x=0.5, y=0.5, z=0.5)
        d.normalize()
        return d

    def getRndPosOnSphere(self, radius=1.0):
        rndVec = radius * self.getRndDir()
        return self + rndVec

# ---------- Molecule Class ----------
class molecule():
    def __init__(self, molecularWeight=12, position=Position()):
        self.MW = molecularWeight
        self.position = position

# ---------- MacroMolecule Class ----------
class macroMolecule():
    def __init__(self, degreeOfPolymerization=1000, segmentLength=0.154E-9, merWt=14):
        self.merWt = merWt
        self.N = int(random.gauss(degreeOfPolymerization, 0.1 * degreeOfPolymerization))
        self.MW = self.N * merWt
        self.segmentLength = segmentLength
        self.centerOfMass = Position()
        self.radiusOfGyration = 0
        self.radiusOfInfluence = 0
        self.endToEndDistance = 0
        self.mers = []

    def freelyJointedChainModel(self):
        lastPosition = Position(x=0, y=0, z=0)
        self.mers = []

        for n in range(self.N):
            m = molecule(molecularWeight=self.merWt)
            m.MW += 1 if (n == 0 or n == (self.N - 1)) else 0
            m.position = lastPosition.getRndPosOnSphere(self.segmentLength)
            self.mers.append(m)
            lastPosition = m.position

        for m in self.mers:
            self.centerOfMass += m.MW * m.position
        self.centerOfMass /= self.MW

        self.endToEndDistance = (self.mers[0].position - self.mers[-1].position).mag()
        self.radiusOfGyration = (
            sum([mer.MW * (mer.position.distTo(self.centerOfMass)) for mer in self.mers]) / (self.MW)
        ) ** 0.5

# ---------- CLI Script ----------
def main():
    try:
        N = int(input("degree of polymerization (1000)?: ") or "1000")
        M = int(input("How many molecules (50)?: ") or "50")
    except ValueError:
        print("Please enter valid integers.")
        return

    centers = []
    end_to_ends = []
    gyrations = []
    molecular_weights = []

    for _ in range(M):
        mol = macroMolecule(degreeOfPolymerization=N)
        mol.freelyJointedChainModel()
        centers.append(mol.centerOfMass.getTup())
        end_to_ends.append(mol.endToEndDistance * 1e6)  # to microns
        gyrations.append(mol.radiusOfGyration * 1e6)     # to microns
        molecular_weights.append(mol.MW)

    avg_com = tuple(round(mean([pos[i] for pos in centers]), 3) for i in range(3))
    avg_ete = round(mean(end_to_ends), 3)
    std_ete = round(stdev(end_to_ends), 3)
    avg_rg = round(mean(gyrations), 3)
    std_rg = round(stdev(gyrations), 3)

    Mw = sum([mw ** 2 for mw in molecular_weights]) / sum(molecular_weights)
    Mn = mean(molecular_weights)
    pdi = round(Mw / Mn, 2)

    print(f"\nMetrics for {M} molecules of degree of polymerization = {N}")
    print(f"\nAvg. Center of Mass (nm) = {avg_com[0]}, {avg_com[1]}, {avg_com[2]}")
    print(f"\nEnd-to-end distance (μm):")
    print(f"\tAverage = {avg_ete}")
    print(f"\tStd. Dev. = {std_ete}")
    print(f"\nRadius of gyration (μm):")
    print(f"\tAverage = {avg_rg}")
    print(f"\tStd. Dev. = {std_rg}")
    print(f"\nPDI = {pdi}")

if __name__ == "__main__":
    main()
