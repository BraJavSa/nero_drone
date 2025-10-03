import numpy as np
from dataclasses import dataclass, field

@dataclass
class LKFParams:
    flag: int = 0
    xpOpt: np.ndarray = field(default_factory=lambda: np.zeros((6,)))
    mseOpt: np.ndarray = field(default_factory=lambda: np.eye(6) * 0.01)
    varwOpt: np.ndarray = field(default_factory=lambda: np.diag([5e-7, 5e-7, 5e-7, 5e-6, 5e-6, 5e-4]))
    varnOpt: np.ndarray = field(default_factory=lambda: np.diag([5e-7, 5e-7, 5e-7, 8e-5, 8e-5, 8e-3]))

@dataclass
class Flags:
    EmergencyStop: int = 0
    isTracked: int = 1

@dataclass
class DroneParams:
    model: str = "Bebop2"
    ip: str = "192.168.0.1"

    Ts: float = 1.0/30.0
    Tsm: float = 1.0/30.0

    g: float = 9.8
    Altmax: float = 2000.0

    # Simplified model vector (will be filled to match MATLAB default)
    Model_simp: np.ndarray = field(default_factory=lambda: np.zeros((8,)))

    # Default controller gains (not all are used by the compensator below, but kept for parity)
    L1: np.ndarray = field(default_factory=lambda: np.eye(6))
    L2: np.ndarray = field(default_factory=lambda: np.diag([1,1,0.5,0.5,0.5,0.5]))
    K:  np.ndarray = field(default_factory=lambda: np.eye(4) * 0.8)

    Ku: np.ndarray = field(default_factory=lambda: np.diag([0.8417, 0.8354, 3.966, 9.8524]))
    Kv: np.ndarray = field(default_factory=lambda: np.diag([0.18227, 0.17095, 4.001, 4.7295]))

    uSat: np.ndarray = field(default_factory=lambda: np.ones((6,)))

    LKF: LKFParams = field(default_factory=LKFParams)
    flags: Flags = field(default_factory=Flags)

def init_parameters(drone_id: int = 1) -> DroneParams:
    p = DroneParams()
    p.ip = f"192.168.0.{drone_id}"
    p.Tsm = p.Ts
    # Mirror MATLAB Model_simp default used by compensator
    p.Model_simp = np.array([0.8417, 0.18227, 0.8354, 0.17095, 3.966, 4.001, 9.8524, 4.7295])
    # Saturations as ones (same intent as MATLAB)
    p.uSat = np.ones((6,))
    return p
