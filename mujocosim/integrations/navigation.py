"""Same-host DDS navigation API, independent of the notebook's interpreter.

Only JSON commands cross RPC. Point clouds use RSim's read-only shared ports.
Client and server must share a host monotonic clock; this is not a network robot
protocol. Goals need a live client lease and never resume after manual takeover.
"""
import asyncio
import json
import os
import time
import uuid

from rsim.runtime.locks import acquire_device
from rsim.runtime.paths import host_id
from rsim.runtime.service import RPCClient, RPCServer, RemoteError
from rsim.runtime.sharing import PortSpec, SharedComponent
from mujocosim.integrations.transport import configure_transport


class NavigationService(RPCServer):
    def __init__(self, name, navigation, *, targets=None, transport=None):
        super().__init__(name, navigation, *((targets,) if targets is not None else ()),
                         transport=configure_transport(transport))
        self.name, self.navigation = name, navigation
        self.targets = targets
        self.sessions = {}
        self.lease = None
        self.quit_requested = False

    async def open(self):
        self.lease = acquire_device(f'mujocosim-navigation:{self.bus.config.domain_id}:{self.name}')
        self.host = host_id()
        await super().open()
        self.task('telemetry', self.broadcast, hz=10.)

    def snapshot(self):
        nav = self.navigation
        result = dict(nav.state.frames[-1].data) if nav.state.frames else {}
        result.update(server_pid=os.getpid(), generation=self.generation,
                      host_id=self.host, quit_requested=self.quit_requested)
        return result

    async def broadcast(self):
        self.telemetry.publish(json.dumps(self.snapshot(), allow_nan=False))

    async def handle(self, request):
        op, nav = request['op'], self.navigation
        if op == 'connect':
            if request.get('host_id') != self.host:
                raise ValueError('navigation demo requires client and simulator on the same host')
            session = uuid.uuid4().hex
            self.sessions[session] = 0
            return dict(session=session, state=self.snapshot())
        session, sequence = request['session'], request['sequence']
        if session not in self.sessions or type(sequence) is not int or sequence <= self.sessions[session]:
            raise ValueError('unknown session or stale command sequence')
        self.sessions[session] = sequence
        if op in ('capture', 'pixel_to_world', 'resolve_pixel', 'pixel_job', 'cancel_pixel_job', 'go_to_pixel', 'topology') and self.targets is None:
            raise ValueError('pixel targeting is not available on this navigation service')
        if op == 'state':
            return self.snapshot()
        if op == 'result':
            return nav.results.get(request['action_id'])
        if op == 'pixel_job':
            return self.targets.job_result(request['job_id'], session)
        if op == 'topology':
            await self.targets.record()
            return self.targets.graph.snapshot()
        if op == 'release':
            if self.targets is not None:
                self.targets.cancel_jobs(session)
            if nav.owner == session:
                nav.cancel('client_closed')
            if nav.manual_owner == session:
                nav.manual(0, 0, owner=session, deadline=0, active=False)
            del self.sessions[session]
            return True
        # All writes, including retries, retain the sender's original deadline.
        deadline = request['deadline_ns']
        remaining = deadline-time.monotonic_ns()
        maximum = .25 if op == 'manual' else .5
        if type(deadline) is not int or not 0 < remaining <= maximum*1e9:
            raise ValueError('command expired or deadline exceeds allowed lease')
        if op == 'capture':
            return await self.targets.capture()
        if op in ('pixel_to_world', 'resolve_pixel'):
            frame = request.get('frame_id')
            if frame is None:
                frame = (await self.targets.capture())['frame_id']
            if op == 'pixel_to_world':
                return self.targets.pixel_to_world(*request['pixel'], frame_id=frame)
            return self.targets.start_resolve(*request['pixel'], frame_id=frame, owner=session)
        if op == 'cancel_pixel_job':
            self.targets.cancel_jobs(session, request['job_id'])
            return True
        if op == 'go_to_pixel':
            return self.targets.go_to_resolved(request['job_id'], owner=session, deadline=deadline)
        if op == 'go_to':
            return nav.go_to(*request['goal'], owner=session, deadline=deadline)
        if op == 'renew':
            return nav.renew(session, deadline)
        if op == 'cancel':
            if self.targets is not None:
                self.targets.cancel_jobs(session)
            nav.cancel()
            return True
        if op == 'manual':
            nav.manual(*request['velocity'], owner=session, deadline=deadline,
                       active=request.get('active', True), brake=request.get('brake', False))
            return True
        raise ValueError('unknown navigation operation')

    async def close(self):
        self.navigation.cancel('service_closed')
        await super().close()
        if self.lease is not None:
            self.lease.close()
            self.lease = None


class NavigationClient(RPCClient):
    """Use inside ``async with Runtime(client)`` (also works in Jupyter).

    ``action = await client.go_to(x, y, yaw)`` returns immediately after ACK.
    ``await client.wait_for_arrival(action)`` awaits arrival, cancellation or failure.
    The active Runtime renews the goal lease even between notebook cells.
    """
    def __init__(self, name='navigation', *, transport=None):
        super().__init__(name, transport=configure_transport(transport))
        self.state = self.signal('state', history=16, clock='host:monotonic')
        self.session = self.generation = self.action_id = None
        self.sequence = 0
        self.last_state = None
        self.frame_id = None

    async def open(self):
        await super().open()
        hello = await self.request(dict(op='hello'), timeout=5.)
        self.generation = hello['generation']
        result = await self.request(dict(op='connect', generation=self.generation, host_id=host_id()), timeout=2.)
        self.session = result['session']
        self.last_state = result['state']
        self.task('state', self.receive_state, hz=20.)
        self.task('lease', self.heartbeat, hz=10.)

    async def call(self, op, *, ttl=.5, timeout=.4, **values):
        self.sequence += 1
        return await self.request(dict(op=op, generation=self.generation,
            session=self.session, sequence=self.sequence,
            deadline_ns=time.monotonic_ns()+int(ttl*1e9), **values), timeout=timeout)

    async def receive_state(self):
        while self.states:
            state = json.loads(self.states.popleft())
            if state['generation'] != self.generation:
                raise RemoteError('navigation provider restarted; reopen the client')
            self.last_state = state
            await self.state.publish(state, stamp_ns=time.monotonic_ns(), clock='host:monotonic')

    async def heartbeat(self):
        if self.action_id is not None:
            action = self.action_id
            renewed = await self.call('renew')
            if not renewed and self.action_id == action:
                self.action_id = None

    async def go_to(self, x, y, yaw=None):
        self.action_id = await self.call('go_to', goal=[float(x), float(y), None if yaw is None else float(yaw)])
        return self.action_id

    async def capture(self):
        """Pin a paired frame. Later pixel calls default to this frame, even after motion."""
        result = await self.call('capture')
        self.frame_id = result['frame_id']
        return result

    async def observe(self, maps, *, timeout=5.):
        """Capture and retrieve the exact RGBD arrays through NavigationMap."""
        capture = await self.capture()
        async with asyncio.timeout(timeout):
            after = None
            while True:
                for frame in maps.observation.frames:
                    if frame.data['frame_id'] == capture['frame_id']:
                        return frame.data
                frame = await maps.observation.get(after=after, timeout=timeout)
                after = frame.sequence

    async def pixel_to_world(self, u, v, *, frame_id=None):
        frame_id = frame_id or self.frame_id or (await self.capture())['frame_id']
        return await self.call('pixel_to_world', pixel=[float(u), float(v)], frame_id=frame_id)

    async def resolve_pixel(self, u, v, *, frame_id=None, timeout=30.):
        """Return a sampled nearest supported, collision-free, connected goal and its path.

        Resolution never moves the robot and does not block server heartbeats.
        """
        frame_id = frame_id or self.frame_id or (await self.capture())['frame_id']
        job = await self.call('resolve_pixel', pixel=[float(u), float(v)], frame_id=frame_id)
        try:
            async with asyncio.timeout(timeout):
                while True:
                    result = await self.call('pixel_job', job_id=job)
                    if result['status'] == 'ready':
                        return result['result']
                    if result['status'] in ('error', 'cancelled'):
                        raise RemoteError(result.get('error', 'pixel resolution cancelled'))
                    await asyncio.sleep(.1)
        except (TimeoutError, asyncio.CancelledError):
            try:
                await self.call('cancel_pixel_job', job_id=job)
            except (TimeoutError, RemoteError):
                pass  # A resolver job alone never actuates the robot.
            raise

    async def go_to_pixel(self, u, v, *, frame_id=None, timeout=30.):
        target = await self.resolve_pixel(u, v, frame_id=frame_id, timeout=timeout)
        result = await self.call('go_to_pixel', job_id=target['job_id'])
        self.action_id = result['action_id']
        return result

    async def navigate_to_pixel(self, u, v, *, frame_id=None, timeout=180.):
        action = await self.go_to_pixel(u, v, frame_id=frame_id)
        result = await self.wait_for_arrival(action['action_id'], timeout=timeout)
        return dict(**result, action_id=action['action_id'], target=action['target'])

    async def get_topology(self):
        return await self.call('topology')

    async def wait_for_arrival(self, action_id, *, timeout=180.):
        try:
            async with asyncio.timeout(timeout):
                while True:
                    result = await self.call('result', action_id=action_id)
                    if result is not None:
                        return result
                    await asyncio.sleep(.1)
        except (TimeoutError, asyncio.CancelledError):
            # A notebook interrupt stops this client's current goal as well.
            if self.action_id == action_id:
                await self.cancel()
            raise

    async def navigate_to(self, x, y, yaw=None, *, timeout=180.):
        return await self.wait_for_arrival(await self.go_to(x, y, yaw), timeout=timeout)

    async def cancel(self):
        self.action_id = None
        return await self.call('cancel')

    async def manual(self, linear=0., angular=0., *, active=True, brake=False, deadline_ns=None):
        self.sequence += 1
        return await self.request(dict(op='manual', generation=self.generation, session=self.session,
            sequence=self.sequence, deadline_ns=deadline_ns or time.monotonic_ns()+250_000_000,
            velocity=[float(linear), float(angular)], active=bool(active), brake=bool(brake)), timeout=.25)

    async def close(self):
        try:
            if self.session is not None:
                await self.call('release', timeout=.3)
        except (TimeoutError, RemoteError):
            pass  # The server also expires this client's original lease.
        finally:
            self.session = self.action_id = None
            await super().close()


class NavigationMap(SharedComponent):
    """Connection-only point cloud and path view; never starts another world."""
    def __init__(self, name='navigation', *, transport=None):
        super().__init__(key='mujocosim-navigation:'+name,
            interface_version='mujocosim-navigation-v1', transport=configure_transport(transport),
            ports={name: PortSpec(history_capacity=size) for name, size in
                   [('map', 2), ('path', 2), ('pose', 32), ('state', 32),
                    ('observation', 2), ('targets', 4), ('topology', 2)]})
