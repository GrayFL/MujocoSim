"""One owner thread for engine access, with independent host-time command safety."""
from __future__ import annotations

import asyncio
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import asdict
from functools import partial
import math
import threading
import time
import uuid

from rsim.core.component import Component
from rsim.core.commands import CommandRejected, CommandSink
from rsim.core.lockstep import StepContext
from rsim.core.pacing import PacerClosed, PacerReset, RatePacer, TickSource
from rsim.runtime.sharing import PortSpec
from .protocol import CommandSpec, Observation


class _PhysicsSuspended(RuntimeError):
    """An explicit control boundary stopped a continuous physics batch."""


class _ResumeRejected(RuntimeError):
    """Expected lifecycle refusal; retain the local RuntimeError contract."""


class MuJoCoSource(Component):
    """Owned backend runtime; all engine operations are serialized on one thread.

    ``ports`` is supplied by the device factory. ``outputs`` remains a narrow
    compatibility argument for the former RSim adapter's injected backends.
    """
    def __init__(self, backend_factory, *, ports=None, outputs=None, commands=None,
                 hz=50., speedup=1., motion_enabled=False, clock='simulation',
                 reset_policy='native', configuration=None, scheduling='realtime',
                 physics_timestep=None, max_substeps=1000, ack_capacity=32):
        super().__init__()
        if not callable(backend_factory):
            raise TypeError('backend_factory must construct a SimulationBackend')
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                   and math.isfinite(v) and v > 0 for v in (hz, speedup)):
            raise ValueError('hz and speedup must be finite and positive')
        if not isinstance(clock, str) or not clock:
            raise ValueError('clock must be a nonempty source time domain')
        if reset_policy not in ('native', 'legacy_continuous'):
            raise ValueError('reset_policy must be native or legacy_continuous')
        if scheduling not in ('realtime', 'accelerated', 'controlled'):
            raise ValueError('scheduling must be realtime, accelerated or controlled')
        if physics_timestep is not None and (not math.isfinite(physics_timestep) or physics_timestep <= 0):
            raise ValueError('physics_timestep must be finite and positive')
        if scheduling == 'controlled' and physics_timestep is None:
            raise ValueError('controlled mode requires an explicit physics_timestep')
        if any(type(value) is not int or value < 1 for value in (max_substeps, ack_capacity)):
            raise ValueError('max_substeps and ack_capacity must be positive integers')
        self.backend_factory, self.clock = backend_factory, clock
        self.hz, self.speedup = hz, speedup
        self.scheduling, self.physics_timestep = scheduling, physics_timestep
        self.max_substeps, self.ack_capacity = max_substeps, ack_capacity
        self.motion_enabled, self.reset_policy = motion_enabled, reset_policy
        self._configuration = configuration
        self.command_specs = dict(commands or {})
        self.backend = None
        self._executor = self._command_lock = self._access = None
        self._commands = {}
        self._envelopes, self._source_deadlines = {}, {}
        self._reset_tasks = set()
        self._transactions = set()
        self._event_tasks = set()
        self._close_task = None
        self._physics_task = self._pacer = self._ticks = None
        self._step_results = OrderedDict()
        self._staged_context = None
        self._phase, self._abort_reason, self._last_result = 'READY', None, None
        self._physical_substeps = 0
        self._state_known = True
        self._physics_not_before_ns = 0
        self.command_expirations = 0
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
            elif name in ('reset_request', 'advance_request', 'inspect_request', 'control_request'):
                port = CommandSink(self, name, getattr(self, '_' + name), fallback=None,
                                   safe=lambda _: None, max_ttl=spec.max_ttl, hz=200)
            else:
                command = self.command_specs[name]
                if not isinstance(command, CommandSpec) or not command.is_safe(spec.fallback):
                    raise ValueError('command fallback must satisfy CommandSpec.is_safe')
                port = CommandSink(self, name, partial(self._apply, name),
                                   fallback=spec.fallback, safe=partial(self._safe, name),
                                   max_ttl=spec.max_ttl, hz=1000 if scheduling == 'accelerated' else 200)
                self._commands[name] = (spec.fallback, 0)
                self._envelopes[name] = None
                self._source_deadlines[name] = 0
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
        guarded = callable(getattr(self.backend, 'step_guarded', None))
        if self.scheduling == 'controlled' and not guarded:
            raise TypeError('controlled backend must provide step_guarded and owner-only stamp_ns')
        if guarded and (self.physics_timestep is None or type(self.backend.stamp_ns) is not int):
            raise TypeError('guarded backend requires physics_timestep and an integer native stamp_ns')

    async def open(self):
        self._command_lock = threading.Lock()
        self._access = asyncio.Lock()
        self._management = asyncio.Lock()
        self._resetting = self._stopping = self._paused = False
        self._reset_failed = False
        self._close_task = None
        self._offset_ns = self._episode = self._step_index = 0
        self._last_stamp_ns = -1
        self._physical_substeps = self.command_expirations = 0
        self._state_known = True
        self._phase, self._abort_reason, self._last_result = 'READY', None, None
        self._step_results.clear()
        self._ticks = None
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='mujocosim-world')
        try:
            await self._work(self._initialize)
            async with self._access:
                await self._publish(await self._work(self._sample, self.demand_snapshot(), True))
            self._start_physics()
        except BaseException:
            await self.close()
            raise

    @property
    def context(self):
        return StepContext(self._instance_id, self._episode, self._step_index)

    def _start_physics(self, *, after_boundary=False):
        if self.scheduling == 'controlled' or self._paused or self._stopping:
            return
        self._physics_not_before_ns = (time.monotonic_ns() + round(1e9/self.hz/self.speedup)
                                       if after_boundary else 0)
        if self.scheduling == 'realtime':
            self._physics_task = self.task('simulation', self._tick, hz=self.hz*self.speedup)
        else:
            self._pacer = RatePacer(self.hz, rate=self.speedup)
            self._physics_task = self.task('simulation', self._tick, pacer=self._pacer)

    async def _stop_physics(self):
        task, self._physics_task = self._physics_task, None
        if task is not None:
            self._tasks.discard(task)
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        self._pacer = None

    def subscribe(self, *, hz=None, policy='latest', capacity=1):
        """Worker-local source events; delivery never implies an algorithm ACK."""
        if self.backend is None or self._stopping:
            raise RuntimeError('source subscriptions bind after the source opens')
        if policy == 'every' and self.scheduling == 'realtime':
            raise ValueError('every-step backpressure requires an accelerated or controlled source')
        if self._ticks is None:
            self._ticks = TickSource(clock=self.clock)
        return self._ticks.subscribe(hz=hz, policy=policy, capacity=capacity)

    @staticmethod
    def _identity(envelope):
        return {name: getattr(envelope, name)
                for name in ('controller_id', 'controller_epoch', 'sequence')}

    async def _apply(self, name, envelope):
        spec = self.command_specs[name]
        if spec.validate:
            spec.validate(envelope.value)
        if not self.motion_enabled and not spec.is_safe(envelope.value):
            raise CommandRejected('active commands require motion_enabled=True')
        generation = (self._instance_id, self._episode)
        async with self._access:
            if generation != (self._instance_id, self._episode):
                raise CommandRejected('command source generation retired while waiting for owner', reason='retired')
            if self._resetting or self._stopping or self._paused or self._phase == 'ABORTED':
                raise CommandRejected('simulation is resetting, paused, stopping or aborted')
            now = time.monotonic_ns()
            if envelope.deadline_ns <= now:
                raise CommandRejected('command expired before staging', reason='expired')
            next_stamp = self._last_stamp_ns + round((self.physics_timestep or 1/self.hz)*1e9)
            if envelope.source_bound is not None:
                if not callable(getattr(self.backend, 'step_guarded', None)):
                    raise CommandRejected('source-bound commands require a guarded backend')
                envelope.source_bound.check(self.context, clock=self.clock, stamp_ns=next_stamp)
            source_deadline = None
            if envelope.source_bound is not None:
                source_deadline = envelope.source_bound.deadline_ns
            elif self.scheduling == 'accelerated':
                source_deadline = self._last_stamp_ns + int((envelope.deadline_ns-now)*self.speedup)
            with self._command_lock:
                self._commands[name] = (envelope.value, envelope.deadline_ns)
                self._envelopes[name] = envelope
                self._source_deadlines[name] = source_deadline
            if self.scheduling == 'controlled':
                self._phase = 'COMMAND_STAGED'
                self._staged_context = self.context
            return dict(kind='CommandAccepted', context=asdict(self.context),
                        command_id=self._identity(envelope), deadline_ns=envelope.deadline_ns,
                        source_deadline_ns=source_deadline, port=name)

    def _safe(self, name, value):
        def clear():
            self._commands[name] = (self.command_specs[name].fallback, 0)
            self._envelopes[name] = None
            self._source_deadlines[name] = 0
        if self._command_lock is None:
            clear()
        else:
            with self._command_lock:
                clear()

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
        guarded = getattr(self.backend, 'step_guarded', None)
        if guarded is None:
            if any(envelope is not None and envelope.source_bound is not None
                   for envelope in self._envelopes.values()):
                raise RuntimeError('source-bound commands require a backend with step_guarded')
            self.backend.step(1/self.hz, self.current_commands)
        else:
            before = self.backend.stamp_ns
            try:
                completed = guarded(1/self.hz, self._commands_for_substep)
            except _PhysicsSuspended:
                self.backend.stop()
                completed = round((self.backend.stamp_ns-before)/(self.physics_timestep*1e9))
            self._physical_substeps += completed
            if not completed:
                return self._sample(demand)
        self._step_index += 1
        return self._sample(demand)

    def _commands_for_substep(self, native_end_ns, *, command_id=None, strict=False):
        """Owner-only check immediately before each complete physical substep."""
        if not strict and (self._paused or self._stopping):
            raise _PhysicsSuspended('continuous source stopped at a physics boundary')
        now, stamp = time.monotonic_ns(), self._offset_ns + native_end_ns
        commands = {}
        with self._command_lock:
            for name, (value, deadline) in self._commands.items():
                envelope = self._envelopes[name]
                valid = envelope is not None and now < deadline
                if command_id is not None:
                    valid &= envelope is not None and self._identity(envelope) == command_id
                if valid and self.scheduling == 'accelerated':
                    valid = stamp <= self._source_deadlines[name]
                if valid and envelope.source_bound is not None:
                    try:
                        envelope.source_bound.check(self.context, clock=self.clock, stamp_ns=stamp)
                    except CommandRejected:
                        valid = False
                if not valid:
                    if strict:
                        raise CommandRejected('staged command identity or host/source interval expired', reason='expired')
                    self.command_expirations += int(envelope is not None)
                    self._commands[name] = (self.command_specs[name].fallback, 0)
                    self._envelopes[name] = None
                    self._source_deadlines[name] = 0
                    value = self.command_specs[name].fallback
                commands[name] = value
        return commands

    async def _publish(self, observation):
        if not isinstance(observation, Observation) or not math.isfinite(observation.time):
            raise ValueError('backend must return Observation with a finite time')
        stamp = self._offset_ns + round(observation.time*1e9)
        if stamp < self._last_stamp_ns:
            raise ValueError('backend time moved backwards without reset')
        self._last_stamp_ns = stamp
        metadata = dict(observation.metadata, episode=self._episode,
                        step_index=self._step_index, session=self._instance_id,
                        source_rate=self.speedup, scheduling=self.scheduling,
                        physical_substeps=self._physical_substeps, phase=self._phase,
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
        if time.monotonic_ns() < self._physics_not_before_ns:
            return
        tick = await self._transaction(self._commit_tick())
        if tick is not None:
            await self._emit_tick(tick)

    async def _commit_tick(self):
        async with self._access:
            if not self._paused and not self._stopping and self._phase != 'ABORTED':
                await self._publish(await self._work(self._advance, self.demand_snapshot()))
                if not self._paused and not self._stopping:
                    return self._tick_fields()

    def _tick_fields(self):
        return dict(stamp_ns=self._last_stamp_ns, session=self._instance_id,
                    episode=self._episode, step_index=self._step_index)

    async def _emit_tick(self, tick):
        # Event backpressure is cancellable after the physics/snapshot commit.
        # Cancelling delivery must never cancel native integration or its ACK.
        if (self._ticks is None or self._stopping or self._paused
                or (tick['session'], tick['episode']) != (self._instance_id, self._episode)):
            return
        task = asyncio.create_task(self._ticks.publish(**tick))
        self._event_tasks.add(task)
        try:
            await task
        except asyncio.CancelledError:
            # Management may cancel just an obsolete delivery. The committed
            # advance caller still receives its ACK; caller cancellation itself
            # remains visible after the native transaction has committed.
            if asyncio.current_task().cancelling():
                raise
        except (PacerClosed, PacerReset):
            pass
        finally:
            self._event_tasks.discard(task)

    async def _cancel_events(self):
        tasks = tuple(self._event_tasks)
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

    async def _transaction(self, coroutine):
        task = asyncio.create_task(coroutine)
        self._transactions.add(task)
        try:
            return await self._finish_transaction(task)
        finally:
            self._transactions.discard(task)

    async def inspect(self, callback, *args):
        """Execute tooling access on the owner; callers must return snapshots."""
        if self._access is None or self._stopping:
            raise RuntimeError('MuJoCo source is not running')
        async with self._access:
            return await self._work(callback, self.backend, *args)

    @staticmethod
    def _parse_context(value):
        if isinstance(value, StepContext):
            value = asdict(value)
        if (not isinstance(value, dict) or set(value) != {'session', 'episode', 'step_index'}
                or not isinstance(value['session'], str) or not value['session']
                or any(type(value[name]) is not int or value[name] < 0
                       for name in ('episode', 'step_index'))):
            raise CommandRejected('context must contain session, episode and step_index', reason='invalid')
        return StepContext(**value)

    @staticmethod
    def _parse_identity(value):
        if (not isinstance(value, dict)
                or set(value) != {'controller_id', 'controller_epoch', 'sequence'}
                or any(not isinstance(value[name], str) or not value[name]
                       for name in ('controller_id', 'controller_epoch'))
                or type(value['sequence']) is not int or value['sequence'] < 1):
            raise CommandRejected('command_id must contain controller_id, controller_epoch and sequence',
                                  reason='invalid')
        return dict(value)

    def _describe(self):
        return dict(context=asdict(self.context), phase=self._phase,
                    stamp_ns=self._last_stamp_ns if self._state_known else None,
                    physical_substeps=self._physical_substeps, state_known=self._state_known,
                    scheduling=self.scheduling, source_rate=self.speedup, paused=self._paused,
                    abort_reason=self._abort_reason, last_result=deepcopy(self._last_result),
                    command_expirations=self.command_expirations,
                    ack_cache_size=len(self._step_results), ack_capacity=self.ack_capacity)

    async def describe(self):
        """Query committed state after any in-flight native transaction finishes."""
        async with self._access:
            return self._describe()

    synchronize = describe

    async def advance(self, context, command_id, requested_substeps, *, _deadline_ns=None):
        """Advance one control boundary by explicit fixed-dt physics substeps.

        Cache lookup precedes current-time checks, so an expired retry can recover
        the original ACK but can never repeat physical integration.
        """
        context = self._parse_context(context)
        command_id = self._parse_identity(command_id)
        if type(requested_substeps) is not int or not 1 <= requested_substeps <= self.max_substeps:
            raise CommandRejected(f'requested_substeps must be an integer in [1, {self.max_substeps}]')
        if self.scheduling != 'controlled':
            raise CommandRejected('advance requires scheduling=controlled')
        result, tick = await self._transaction(self._advance_step(
            context, command_id, requested_substeps, _deadline_ns))
        if tick is not None:
            await self._emit_tick(tick)
        return result

    def _cache_step(self, context, signature, result, reason=None):
        self._step_results[context] = (signature, deepcopy(result), reason)
        while len(self._step_results) > self.ack_capacity:
            self._step_results.popitem(last=False)
        self._last_result = deepcopy(result)

    def _advance_native(self, command_id, requested_substeps, deadline_ns, demand):
        before = self.backend.stamp_ns
        error, observation = None, None
        def commands(end_stamp_ns):
            if deadline_ns is not None and time.monotonic_ns() >= deadline_ns:
                raise CommandRejected('advance request expired on owner', reason='expired')
            return self._commands_for_substep(end_stamp_ns, command_id=command_id, strict=True)
        try:
            self.backend.step_guarded(requested_substeps*self.physics_timestep, commands)
        except BaseException as failure:
            error = failure
            try:
                self.backend.stop()
            except BaseException as stop_error:
                error = RuntimeError(f'{failure}; safe stop also failed: {stop_error}')
        stamp = self.backend.stamp_ns
        completed = round((stamp-before)/(self.physics_timestep*1e9))
        try:
            observation = self._sample(demand)
        except BaseException as failure:
            error = failure
            try:
                self.backend.stop()
            except BaseException as stop_error:
                error = RuntimeError(f'{failure}; safe stop also failed: {stop_error}')
        return observation, stamp, completed, error

    async def _advance_step(self, context, command_id, requested_substeps, deadline_ns):
        signature = (tuple(sorted(command_id.items())), requested_substeps)
        async with self._access:
            cached = self._step_results.get(context)
            if cached is not None:
                previous_signature, result, reason = cached
                if previous_signature != signature:
                    raise CommandRejected('step identity was already used with different contents')
                if reason is not None:
                    raise CommandRejected(result['error'], reason=reason)
                return deepcopy(result), None
            if context != self.context:
                raise CommandRejected('step context is stale, unknown or no longer cached', reason='retired')
            if self._stopping or self._paused or self._phase == 'ABORTED':
                raise CommandRejected('source is stopping, paused or aborted; reset before advancing')
            error, observation, completed = None, None, 0
            stamp = self._last_stamp_ns - self._offset_ns
            if self._phase != 'COMMAND_STAGED' or self._staged_context != context:
                error = CommandRejected('this control step has no staged command')
            elif deadline_ns is not None and deadline_ns <= time.monotonic_ns():
                error = CommandRejected('advance request expired before owner execution', reason='expired')
            else:
                self._phase = 'ADVANCING'
                try:
                    observation, stamp, completed, error = await self._work(
                        self._advance_native, command_id, requested_substeps, deadline_ns, self.demand_snapshot())
                except BaseException as failure:
                    # An unavailable executor or unreadable native state makes
                    # partial progress unknown, never implicitly zero.
                    error, stamp, completed = failure, None, None
                    self._state_known = False
            self._physical_substeps = (None if completed is None else self._physical_substeps + completed)
            if error is None and completed != requested_substeps:
                error = RuntimeError('backend did not complete all requested physical substeps')
            if error is None:
                self._step_index += 1
                self._phase = 'READY'
                try:
                    await self._publish(observation)
                except BaseException as failure:
                    error = failure
                    self._step_index = context.step_index
            if error is not None:
                self._phase, self._abort_reason = 'ABORTED', str(error)
                self._invalidate_commands()
                try:
                    await self._work(self.backend.stop)
                except BaseException as stop_error:
                    self._abort_reason = f'{error}; safe stop also failed: {stop_error}'
                if stamp is not None:
                    self._last_stamp_ns = self._offset_ns + stamp
                # Publish partial physical state when possible; if publication
                # itself failed, discard snapshots instead of exposing a mix.
                try:
                    if not self._state_known:
                        raise RuntimeError('native state is unknown after owner failure')
                    if observation is None:
                        observation = await self._work(self._sample, self.demand_snapshot(), True)
                    await self._publish(observation)
                except BaseException:
                    for port in self.outputs.values():
                        await port.clear_history()
                result = dict(kind='PhysicsAborted', request_context=asdict(context),
                              context=asdict(self.context), command_id=command_id,
                              requested_substeps=requested_substeps, actual_substeps=completed,
                              physical_substeps=self._physical_substeps,
                              stamp_ns=self._last_stamp_ns if self._state_known else None,
                              state_known=self._state_known, phase='ABORTED', error=self._abort_reason,
                              state=self._state_summary(observation))
                reason = getattr(error, 'reason', 'invalid')
                self._cache_step(context, signature, result, reason)
                raise CommandRejected(str(error), reason=reason) from error
            result = dict(kind='PhysicsAdvanced', request_context=asdict(context),
                          context=asdict(self.context), command_id=command_id,
                          requested_substeps=requested_substeps, actual_substeps=completed,
                          physical_substeps=self._physical_substeps, stamp_ns=self._last_stamp_ns,
                          phase='READY', state=self._state_summary(observation))
            self._cache_step(context, signature, result)
            return deepcopy(result), self._tick_fields()

    @staticmethod
    def _state_summary(observation):
        return deepcopy(observation.values.get('state', {})) if observation is not None else None

    async def _advance_request(self, envelope):
        value = envelope.value
        if value is None:
            return None
        if not isinstance(value, dict) or set(value) != {'context', 'command_id', 'requested_substeps'}:
            raise CommandRejected('advance request requires context, command_id and requested_substeps')
        try:
            return await self.advance(**value, _deadline_ns=envelope.deadline_ns)
        except CommandRejected as error:
            if error.reason == 'retired':
                # This is a source step identity, not the independent controller
                # epoch of the management transport. Do not retire that channel
                # when its caller queries an old/unknown physical transaction.
                raise CommandRejected(str(error), reason='stale_context') from error
            raise

    async def _inspect_request(self, envelope):
        if envelope.value is None:
            return None
        if envelope.value != {}:
            raise CommandRejected('inspect request must be an empty mapping')
        return await self.describe()

    async def _control_request(self, envelope):
        value = envelope.value
        if value is None:
            return None
        if not isinstance(value, dict) or set(value) - {'operation', 'rate', 'reason'}:
            raise CommandRejected('control request must be a mapping with operation')
        operation = value.get('operation')
        if operation not in ('pause', 'resume', 'set_rate', 'abort'):
            raise CommandRejected('unknown simulation control operation')
        if operation == 'set_rate':
            return await self.set_rate(value.get('rate'), _deadline_ns=envelope.deadline_ns)
        if operation == 'abort':
            return await self.abort(value.get('reason', 'caller aborted'), _deadline_ns=envelope.deadline_ns)
        try:
            return await getattr(self, operation)(_deadline_ns=envelope.deadline_ns)
        except _ResumeRejected as error:
            # A rejected transition is a command response. Other RuntimeErrors
            # remain owner failures and must not be hidden by this boundary.
            raise CommandRejected(str(error)) from error

    def _invalidate_commands(self):
        self._staged_context = None
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
        async with self._management:
            self._check_deadline(_deadline_ns)
            await self._stop_physics()
            await self._cancel_events()
            try:
                result = await self._reset_locked(*args, _deadline_ns=_deadline_ns, **kwargs)
                await self._emit_tick(self._tick_fields())
                return result
            finally:
                self._start_physics(after_boundary=True)

    async def _reset_locked(self, *args, _deadline_ns=None, **kwargs):
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
                self._physical_substeps = 0
                self._state_known = True
                self._step_results.clear()
                self._phase, self._abort_reason, self._last_result = 'READY', None, None
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
                    self._phase, self._abort_reason = 'ABORTED', 'reset or initial snapshot failed'
                    self._state_known, self._physical_substeps = False, None
                    self._step_results.clear()
                    self._last_result = None
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

    @staticmethod
    def _check_deadline(deadline_ns):
        if deadline_ns is not None and deadline_ns <= time.monotonic_ns():
            raise CommandRejected('management request expired', reason='expired')

    async def pause(self, *, _deadline_ns=None):
        result, _ = await self._transaction(self._control('pause', deadline_ns=_deadline_ns))
        return result

    async def resume(self, *, _deadline_ns=None):
        result, tick = await self._transaction(self._control('resume', deadline_ns=_deadline_ns))
        if tick is not None:
            await self._emit_tick(tick)
        return result

    async def set_rate(self, rate, *, _deadline_ns=None):
        if not isinstance(rate, (int, float)) or isinstance(rate, bool) or not math.isfinite(rate) or rate <= 0:
            raise CommandRejected('source rate must be finite and positive')
        if self.scheduling != 'accelerated':
            raise CommandRejected('set_rate requires scheduling=accelerated')
        result, tick = await self._transaction(self._control('set_rate', rate=rate, deadline_ns=_deadline_ns))
        if tick is not None:
            await self._emit_tick(tick)
        return result

    async def abort(self, reason='caller aborted', *, _deadline_ns=None):
        if not isinstance(reason, str) or not reason:
            raise CommandRejected('abort reason must be a nonempty string')
        result, _ = await self._transaction(self._control('abort', reason=reason, deadline_ns=_deadline_ns))
        return result

    async def _new_episode(self):
        self._invalidate_commands()
        self._episode += 1
        self._phase, self._abort_reason, self._last_result = 'READY', None, None
        self._step_results.clear()
        for port in self.outputs.values():
            await port.clear_history()
        await self._publish(await self._work(self._sample, self.demand_snapshot(), True))

    async def _control(self, operation, *, rate=None, reason=None, deadline_ns=None):
        async with self._management:
            self._check_deadline(deadline_ns)
            if operation == 'resume' and (self._reset_failed or self._phase == 'ABORTED'):
                raise _ResumeRejected('source aborted; complete a successful reset before resume')
            if operation == 'set_rate' and self._phase == 'ABORTED':
                raise CommandRejected('source aborted; reset before changing the rate')
            if operation == 'resume' and not self._paused:
                return self._describe(), None
            if operation == 'set_rate' and rate == self.speedup:
                return self._describe(), None
            if operation == 'set_rate' and self._paused:
                self.speedup = rate
                return self._describe(), None
            if operation in ('pause', 'abort'):
                self._paused = True
            # Retire on the event loop immediately; an owner between substeps
            # sees safe commands even while management waits for its turn.
            self._invalidate_commands()
            await self._stop_physics()
            await self._cancel_events()
            tick = None
            async with self._access:
                try:
                    await self._work(self.backend.stop)
                    if operation == 'abort':
                        self._phase, self._abort_reason = 'ABORTED', reason
                    elif operation == 'pause':
                        if self._phase != 'ABORTED':
                            self._phase = 'READY'
                    else:
                        self._check_deadline(deadline_ns)
                        if operation == 'set_rate':
                            self.speedup = rate
                        self._paused = False
                        await self._new_episode()
                        tick = self._tick_fields()
                except BaseException as error:
                    self._paused = True
                    self._phase, self._abort_reason = 'ABORTED', str(error)
                    for port in self.outputs.values():
                        await port.clear_history()
                    raise CommandRejected(str(error), reason=getattr(error, 'reason', 'invalid')) from error
                result = self._describe()
            self._start_physics(after_boundary=True)
            return result, tick

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
        if self._ticks is not None:
            await self._ticks.aclose()
        await self._cancel_events()
        await self._stop_physics()
        pending = tuple(self._reset_tasks | self._transactions)
        if pending:
            await asyncio.gather(*pending, return_exceptions=True)
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
