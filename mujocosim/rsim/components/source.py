"""One owner thread for engine access, with independent host-time command safety."""
from __future__ import annotations

import asyncio
from concurrent.futures import ThreadPoolExecutor
from functools import partial
import math
import threading
import time
import uuid

from rsim.core.component import Component
from rsim.core.commands import CommandRejected, CommandSink
from rsim.runtime.sharing import PortSpec
from .protocol import CommandSpec, Observation


class MuJoCoSource(Component):
    """Owned backend runtime; all engine operations are serialized on one thread.

    ``ports`` is supplied by the device factory. ``outputs`` remains a narrow
    compatibility argument for the former RSim adapter's injected backends.
    """
    def __init__(self, backend_factory, *, ports=None, outputs=None, commands=None,
                 hz=50., speedup=1., motion_enabled=False, clock='simulation',
                 reset_policy='native', configuration=None):
        super().__init__()
        if not callable(backend_factory):
            raise TypeError('backend_factory must construct a SimulationBackend')
        if not all(math.isfinite(v) and v > 0 for v in (hz, speedup)):
            raise ValueError('hz and speedup must be finite and positive')
        if not isinstance(clock, str) or not clock:
            raise ValueError('clock must be a nonempty source time domain')
        if reset_policy not in ('native', 'legacy_continuous'):
            raise ValueError('reset_policy must be native or legacy_continuous')
        self.backend_factory, self.clock = backend_factory, clock
        self.hz, self.speedup = hz, speedup
        self.motion_enabled, self.reset_policy = motion_enabled, reset_policy
        self._configuration = configuration
        self.command_specs = dict(commands or {})
        self.backend = None
        self._executor = self._command_lock = self._access = None
        self._commands = {}
        self._reset_tasks = set()
        self._close_task = None
        self._reset_failed = False
        self._resetting = self._stopping = self._paused = False
        self._episode = self._step_index = self._offset_ns = 0
        self._last_stamp_ns = -1
        if ports is None:
            ports = {name: PortSpec(clock=clock, history_capacity=history)
                     for name, history in (outputs or {}).items()}
            ports.update({name: PortSpec(direction='sink', clock='host:monotonic',
                                         max_ttl=.5, fallback=spec.fallback)
                          for name, spec in self.command_specs.items()})
        elif outputs is not None:
            raise ValueError('supply ports or legacy outputs, not both')
        self.port_specs = dict(ports)
        if any(not name.isidentifier() or hasattr(self, name) for name in ports):
            raise ValueError('port names must be identifiers without Component conflicts')
        for name, spec in self.port_specs.items():
            if spec.direction == 'signal':
                port = self.signal(name, history=spec.history_capacity, clock=spec.clock)
            elif name == 'reset_request':
                port = CommandSink(self, name, self._reset_request, fallback=None,
                                   safe=lambda _: None, max_ttl=spec.max_ttl, hz=200)
            else:
                command = self.command_specs[name]
                if not isinstance(command, CommandSpec) or not command.is_safe(spec.fallback):
                    raise ValueError('command fallback must satisfy CommandSpec.is_safe')
                port = CommandSink(self, name, partial(self._apply, name),
                                   fallback=spec.fallback, safe=partial(self._safe, name),
                                   max_ttl=spec.max_ttl, hz=200)
                self._commands[name] = (spec.fallback, 0)
            setattr(self, name, port)

    def configuration(self):
        return super().configuration(), self._configuration

    async def _work(self, callback, *args):
        if self._executor is None:
            raise RuntimeError('MuJoCo source is not running')
        return await asyncio.get_running_loop().run_in_executor(self._executor, callback, *args)

    def _initialize(self):
        self.backend = self.backend_factory()
        self.backend.open()

    async def open(self):
        self._command_lock = threading.Lock()
        self._access = asyncio.Lock()
        self._resetting = self._stopping = self._paused = False
        self._reset_failed = False
        self._close_task = None
        self._offset_ns = self._episode = self._step_index = 0
        self._last_stamp_ns = -1
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='mujocosim-world')
        try:
            await self._work(self._initialize)
            async with self._access:
                await self._publish(await self._work(self._sample, self.demand_snapshot(), True))
            self.task('simulation', self._tick, hz=self.hz*self.speedup)
        except BaseException:
            await self.close()
            raise

    def _apply(self, name, envelope):
        spec = self.command_specs[name]
        if spec.validate:
            spec.validate(envelope.value)
        if self._resetting or self._stopping or self._paused:
            raise CommandRejected('simulation is resetting, paused or stopping')
        if not self.motion_enabled and not spec.is_safe(envelope.value):
            raise CommandRejected('active commands require motion_enabled=True')
        with self._command_lock:
            self._commands[name] = (envelope.value, envelope.deadline_ns)

    def _safe(self, name, value):
        if self._command_lock is None:
            self._commands[name] = (self.command_specs[name].fallback, 0)
        else:
            with self._command_lock:
                self._commands[name] = (self.command_specs[name].fallback, 0)

    @property
    def current_commands(self):
        now = time.monotonic_ns()
        def snapshot():
            return {name: value if now < deadline else self.command_specs[name].fallback
                    for name, (value, deadline) in self._commands.items()}
        if self._command_lock is None:
            return snapshot()
        with self._command_lock:
            return snapshot()

    def _sample(self, demand, force=False):
        return self.backend.sample(demand, force=force)

    def _advance(self, demand):
        # This check occurs on the owner immediately before the physical action.
        # Waiting for the event loop or accelerating physics cannot extend TTL.
        self.backend.step(1/self.hz, self.current_commands)
        self._step_index += 1
        return self._sample(demand)

    async def _publish(self, observation):
        if not isinstance(observation, Observation) or not math.isfinite(observation.time):
            raise ValueError('backend must return Observation with a finite time')
        stamp = self._offset_ns + round(observation.time*1e9)
        if stamp < self._last_stamp_ns:
            raise ValueError('backend time moved backwards without reset')
        self._last_stamp_ns = stamp
        metadata = dict(observation.metadata, episode=self._episode,
                        step_index=self._step_index, session=self._instance_id,
                        host_monotonic_ns=time.monotonic_ns())
        for name, value in observation.values.items():
            if name not in self.outputs:
                raise ValueError(f'backend returned undeclared output: {name}')
            port_metadata = dict(metadata, **observation.port_metadata.get(name, {}))
            if name == 'state' and isinstance(value, dict):
                value = dict(value, episode=self._episode, step_index=self._step_index)
            await self.outputs[name].publish(value, stamp_ns=stamp, clock=self.clock,
                                             metadata=port_metadata)

    async def _tick(self):
        async with self._access:
            if not self._paused and not self._stopping:
                await self._publish(await self._work(self._advance, self.demand_snapshot()))

    async def inspect(self, callback, *args):
        """Execute tooling access on the owner; callers must return snapshots."""
        if self._access is None or self._stopping:
            raise RuntimeError('MuJoCo source is not running')
        async with self._access:
            return await self._work(callback, self.backend, *args)

    def _invalidate_commands(self):
        for name, spec in self.command_specs.items():
            self._safe(name, spec.fallback)
            sink = self.sinks[name]
            sink.guard.retire()
            sink._armed = False
            # Local direct commands get a new session. Remote controllers must
            # explicitly start a new epoch after a reset; old frames stay retired.
            sink._epoch, sink._sequence = uuid.uuid4().hex, 0

    async def reset(self, *args, **kwargs):
        # A cancelled request must still finish the episode transition once the
        # owner has begun resetting; the next tick cannot observe half a reset.
        if self._access is None or self._stopping:
            raise RuntimeError('MuJoCo source is not running')
        task = asyncio.create_task(self._reset(*args, **kwargs))
        self._reset_tasks.add(task)
        try:
            return await self._finish_transaction(task)
        finally:
            self._reset_tasks.discard(task)

    @staticmethod
    async def _finish_transaction(task):
        """Keep native cleanup isolated from any number of caller cancellations."""
        cancelled = False
        while True:
            try:
                result = await asyncio.shield(task)
                break
            except asyncio.CancelledError:
                if task.cancelled():
                    raise
                cancelled = True
        if cancelled:
            raise asyncio.CancelledError
        return result

    async def _reset(self, *args, _deadline_ns=None, **kwargs):
        if self._access is None or self._stopping:
            raise RuntimeError('MuJoCo source is not running')
        async with self._access:
            if _deadline_ns is not None and _deadline_ns <= time.monotonic_ns():
                raise CommandRejected('reset request expired', reason='expired')
            self._resetting = True
            self._invalidate_commands()
            demand = self.demand_snapshot()
            previous_episode = self._episode
            started = False
            def reset_backend():
                nonlocal started
                if _deadline_ns is not None and _deadline_ns <= time.monotonic_ns():
                    raise CommandRejected('reset request expired', reason='expired')
                started = True
                self.backend.stop()
                self.backend.reset(*args, **kwargs)
                return self._sample(demand, True)
            try:
                observation = await self._work(reset_backend)
                previous_stamp = self._last_stamp_ns
                self._episode += 1
                self._step_index = 0
                if self.reset_policy == 'legacy_continuous':
                    self._offset_ns = (previous_stamp + max(1, round(1e9/self.hz))
                                       - round(observation.time*1e9))
                else:
                    self._offset_ns = 0
                self._last_stamp_ns = -1
                for port in self.outputs.values():
                    await port.clear_history()
                await self._publish(observation)
                self._reset_failed = False
                return {'episode': self._episode, 'step_index': self._step_index}
            except BaseException:
                # The backend may already have reset even if sampling failed.
                # Never resume from a snapshot of the previous physical world.
                if started:
                    self._paused = self._reset_failed = True
                    self._episode = previous_episode + 1
                    self._step_index = 0
                    self._last_stamp_ns = -1
                    for port in self.outputs.values():
                        await port.clear_history()
                raise
            finally:
                self._resetting = False

    async def _reset_request(self, envelope):
        value = envelope.value
        if value is None:
            return None
        if not isinstance(value, dict) or set(value) - {'spawn'}:
            raise CommandRejected('reset request must be a mapping with optional spawn')
        if envelope.deadline_ns <= time.monotonic_ns():
            raise CommandRejected('reset request expired', reason='expired')
        try:
            return await self.reset(spawn=value.get('spawn'), _deadline_ns=envelope.deadline_ns)
        except ValueError as error:
            if isinstance(error, CommandRejected):
                raise
            raise CommandRejected(str(error)) from error

    async def pause(self):
        async with self._access:
            self._paused = True
            self._invalidate_commands()
            await self._work(self.backend.stop)

    async def resume(self):
        async with self._access:
            if self._reset_failed:
                raise RuntimeError('reset failed; complete a successful reset before resume')
            self._paused = False

    async def close(self):
        if self._executor is None:
            return
        if self._close_task is None:
            self._stopping = True
            self._invalidate_commands()
            self._close_task = asyncio.create_task(self._close_resources())
        await self._finish_transaction(self._close_task)

    async def _close_resources(self):
        # Public reset requests need not be Component tasks. Join their native
        # operation and episode publication before destroying the engine.
        if self._reset_tasks:
            await asyncio.gather(*tuple(self._reset_tasks), return_exceptions=True)
        def cleanup():
            if self.backend is not None:
                try:
                    self.backend.stop()
                finally:
                    self.backend.close()
        try:
            await self._work(cleanup)
        finally:
            self._executor.shutdown(wait=True)
            self._executor = None


class SimulationSource(MuJoCoSource):
    """Differential-drive World facade with serialized inspection."""
    @property
    def prepared_scene(self):
        return self.backend.prepared_scene

    @property
    def command(self):
        return self.current_commands['velocity']

    async def inspect(self, callback, *args):
        return await super().inspect(lambda backend: callback(backend.simulation, *args))


class LegacySimulationSource(SimulationSource):
    """Transition-only direct engine view for pre-0.3 tooling (retired in 0.4)."""
    @property
    def simulation(self):
        return self.backend.simulation if self.backend else None
