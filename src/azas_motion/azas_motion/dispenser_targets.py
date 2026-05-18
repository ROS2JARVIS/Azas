"""Pure target calculations for dispenser-front MoveIt demos."""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Protocol


@dataclass(frozen=True)
class Position:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class DispenserTarget:
    dispenser_id: int
    label: str
    position: Position


class PoseLike(Protocol):
    position: Position


def parse_outlet_positions(values: list[float]) -> list[Position]:
    outlets = [float(value) for value in values]
    if len(outlets) % 3 != 0:
        raise ValueError("dispenser_outlet_positions must be a flat XYZ list")
    return [
        Position(outlets[index], outlets[index + 1], outlets[index + 2])
        for index in range(0, len(outlets), 3)
    ]


def selected_outlet(outlets: list[Position], selected_id: int) -> Position:
    if not outlets:
        raise ValueError("dispenser_outlet_positions must contain at least one outlet")
    index = min(max(int(selected_id), 1), len(outlets)) - 1
    return outlets[index]


def dispenser_front_targets(
    dispenser_id: int,
    outlet: Position,
    *,
    front_approach_offset_x: float,
    outlet_front_offset_x: float,
    transfer_z_override: float,
    detour_y: float,
    enable_obstacle_detour: bool,
    prefix: str = "",
) -> list[DispenserTarget]:
    hold_x = outlet.x - outlet_front_offset_x
    transfer_z = transfer_z_override if transfer_z_override > 0.0 else outlet.z
    label_prefix = f"{prefix}dispenser_{dispenser_id}_"
    hold = Position(hold_x, outlet.y, transfer_z)

    if not enable_obstacle_detour:
        return [DispenserTarget(dispenser_id, f"{label_prefix}outlet_front_hold", hold)]

    approach_x = outlet.x - front_approach_offset_x
    return [
        DispenserTarget(
            dispenser_id,
            f"{label_prefix}obstacle_detour",
            Position(approach_x, detour_y, transfer_z),
        ),
        DispenserTarget(
            dispenser_id,
            f"{label_prefix}outlet_approach",
            Position(hold_x, detour_y, transfer_z),
        ),
        DispenserTarget(dispenser_id, f"{label_prefix}outlet_front_hold", hold),
    ]


def nearest_dispenser_order(
    sequence_ids: list[int],
    outlets: list[Position],
    current_position: Position,
    *,
    outlet_front_offset_x: float,
    transfer_z_override: float,
) -> list[int]:
    remaining = list(sequence_ids)
    ordered: list[int] = []
    cursor = current_position
    while remaining:
        nearest_id = min(
            remaining,
            key=lambda dispenser_id: distance(
                cursor,
                hold_position(
                    outlets[dispenser_id - 1],
                    outlet_front_offset_x=outlet_front_offset_x,
                    transfer_z_override=transfer_z_override,
                ),
            ),
        )
        ordered.append(nearest_id)
        remaining.remove(nearest_id)
        cursor = hold_position(
            outlets[nearest_id - 1],
            outlet_front_offset_x=outlet_front_offset_x,
            transfer_z_override=transfer_z_override,
        )
    return ordered


def hold_position(
    outlet: Position,
    *,
    outlet_front_offset_x: float,
    transfer_z_override: float,
) -> Position:
    transfer_z = transfer_z_override if transfer_z_override > 0.0 else outlet.z
    return Position(outlet.x - outlet_front_offset_x, outlet.y, transfer_z)


def distance(first: Position, second: Position) -> float:
    return sqrt(
        (first.x - second.x) ** 2
        + (first.y - second.y) ** 2
        + (first.z - second.z) ** 2
    )
