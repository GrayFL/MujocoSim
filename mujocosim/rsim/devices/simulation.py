"""Scene-free JSON management view; importing it never imports the engine."""
from dataclasses import asdict
from rsim.core.lockstep import StepContext
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

    async def advance(self, context, command_id, requested_substeps, *, ttl=None):
        """Return PhysicsAdvanced only after fixed-dt integration and publication.

        Retry the same three-field request or synchronize after a missing ACK;
        a timeout does not imply that physics stayed at the old boundary.
        """
        if isinstance(context, StepContext):
            context = asdict(context)
        return await self.advance_request.set(dict(context=context, command_id=command_id,
                                                   requested_substeps=requested_substeps), ttl=ttl)

    async def describe(self, *, ttl=None):
        return await self.inspect_request.set({}, ttl=ttl)

    synchronize = describe

    async def pause(self, *, ttl=None):
        return await self.control_request.set({'operation': 'pause'}, ttl=ttl)

    async def resume(self, *, ttl=None):
        return await self.control_request.set({'operation': 'resume'}, ttl=ttl)

    async def set_rate(self, rate, *, ttl=None):
        return await self.control_request.set({'operation': 'set_rate', 'rate': rate}, ttl=ttl)

    async def abort(self, reason='caller aborted', *, ttl=None):
        return await self.control_request.set({'operation': 'abort', 'reason': reason}, ttl=ttl)
