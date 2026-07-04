"""

Coordinate Frame:
-----------------
World Frame (X,Y)

Robot always approaches cubes facing +X
Robot always approaches bins facing -X

"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Location:
    x: float
    y: float


# ==========================================================
# PICKUP LOCATIONS
# Robot Base Center Position
# ==========================================================

RED_PICKUP = Location(
    x=1.125,
    y=-1.5
)

GREEN_PICKUP = Location(
    x=1.125,
    y=0.0
)

BLUE_PICKUP = Location(
    x=1.125,
    y=1.5
)


# ==========================================================
# DROP LOCATIONS
# Robot Base Center Position
# ==========================================================

RED_DROP = Location(
    x=-0.8,
    y=-1.5
)

GREEN_DROP = Location(
    x=-0.8,
    y=0.0
)

BLUE_DROP = Location(
    x=-0.8,
    y=1.5
)


# ==========================================================
# WORLD LOCATIONS
# Actual Cube Locations
# (Useful for debugging)
# ==========================================================

RED_BOX = Location(
    x=2.0,
    y=-1.5
)

GREEN_BOX = Location(
    x=2.0,
    y=0.0
)

BLUE_BOX = Location(
    x=2.0,
    y=1.5
)


RED_BIN = Location(
    x=-2.0,
    y=-1.5
)

GREEN_BIN = Location(
    x=-2.0,
    y=0.0
)

BLUE_BIN = Location(
    x=-2.0,
    y=1.5
)

# ==========================================================
# CAMERA INSPECTION LOCATIONS
# Robot moves here before detecting the next cube
# ==========================================================

INSPECTION_CENTER = Location(
    x=0.0,
    y=0.0
)

INSPECTION_LEFT = Location(
    x=0.0,
    y=1.5
)

INSPECTION_RIGHT= Location(
    x=0.0,
    y=-1.5
)

# ==========================================================
# Dictionary Lookup
# ==========================================================

PICKUP_LOCATIONS = {
    "red": RED_PICKUP,
    "green": GREEN_PICKUP,
    "blue": BLUE_PICKUP
}

DROP_LOCATIONS = {
    "red": RED_DROP,
    "green": GREEN_DROP,
    "blue": BLUE_DROP
}

BOX_LOCATIONS = {
    "red": RED_BOX,
    "green": GREEN_BOX,
    "blue": BLUE_BOX
}

BIN_LOCATIONS = {
    "red": RED_BIN,
    "green": GREEN_BIN,
    "blue": BLUE_BIN
}

