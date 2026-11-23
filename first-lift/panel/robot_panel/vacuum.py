"""Packets for vacuum pumps and valves."""

from __future__ import annotations

from .command_codes import CMD_PUMPS, CMD_VALVES


class VacuumCommands:
    def pumps(self, gripper: int, mask: int) -> None:
        """0x21: gripper byte + four-bit pump mask."""
        self.set_mechanism(CMD_PUMPS, bytes((gripper, mask)))

    def valves(self, gripper: int, mask: int) -> None:
        """0x22: gripper byte + four-bit valve mask."""
        self.set_mechanism(CMD_VALVES, bytes((gripper, mask)))

    def suction(
        self, gripper: int, mask: int, enabled: bool, action: str | None = None
    ) -> None:
        label = action or f"Gripper {gripper}: suction {'ON' if enabled else 'OFF'}"
        if enabled:
            self.set_mechanism(
                CMD_VALVES, bytes((gripper, mask)), action=label + " / open valves"
            )
            self.set_mechanism(
                CMD_PUMPS, bytes((gripper, mask)), action=label + " / start pumps"
            )
        else:
            self.set_mechanism(
                CMD_PUMPS, bytes((gripper, 0)), action=label + " / stop pumps"
            )
            self.set_mechanism(
                CMD_VALVES, bytes((gripper, 0)), action=label + " / close valves"
            )
