"""Scene-free connection view with acknowledged owner-thread reset."""
from rsim.runtime.sharing import SharedComponent
from mujocosim.rsim.drivers.chassis import MujocoChassisFactory


class SimulationDevice(SharedComponent):
    def __init__(self, *, key, clock='simulation', history=32, image_history=8,
                 max_ttl=.5, transport=None, options=None):
        super().__init__(key=key,
                         ports=MujocoChassisFactory.port_specs(clock, history=history,
                                    image_history=image_history, max_ttl=max_ttl),
                         interface_version=MujocoChassisFactory.INTERFACE_VERSION,
                         transport=transport, options=options)

    async def reset(self, spawn=None, *, ttl=None):
        """Acknowledge only after reset and first-frame publication.

        Motion controllers must start a new controller epoch afterwards.
        """
        return await self.reset_request.set({'spawn': None if spawn is None else tuple(spawn)}, ttl=ttl)
