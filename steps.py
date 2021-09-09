"""This module contains the @plan and Step interfaces.

It is passed as the global environment for plan generation extensions."""

from sys import api_version
from errors import *

class AbortStep(Exception): pass

class StepList:
    """Holds steps in a tree structure.

    The order that steps are run is the order that they are added to the tree;
    the tree is just used to organize steps into human-relevant blocks to 
    aid visualization.
    """

    class Block:
        """Helper class; one block/branch."""
        def __init__(self, parent, name):
            self.name = name
            self.parent = parent
            self.children = []
        
        def serialize(self):
            l = []
            for x in self.children:
                if isinstance(x, StepList.Block):
                    l += x.serialize()
                else:
                    l.append(x)
            return l

    def __init__(self):
        """Initialize with an empty root."""
        self.root = self.Block(None, 'root')
        self.current_block = self.root
    
    @classmethod
    def from_list(cls, l):
        """Fill the root block with the given steps."""
        obj = cls()
        obj.root.children += l
        return obj
    
    def append(self, step):
        """Append `step` to the current block."""
        self.current_block.children.append(step)
    
    def __iadd__(self, steps):
        """Append `steps` to the current block."""
        self.current_block.children += steps
        return self
    
    def push(self, name):
        """Enter a new block, with the given name."""
        self.current_block.children.append(
            self.Block(self.current_block, name))
    
    def pop(self):
        """Exit the block, going up one level."""
        if self.current_block is not self.root:
            self.current_block = self.current_block.parent
    
    def serialize(self):
        """Serialize the tree into a list of steps to run in order."""
        return self.root.serialize()

class Step:
    """Represent one reproducible experiment step."""
    def run(self, olf):
        """Run the step, using the given Olfactometer instance.

        If the step is 'atomic' (cannot be canceled), it can be written
        as a regular function.

        If the step can be cancelled, it should be written as a generator
        function that yields at acceptable cancel points. Most plans of this
        type need to sleep for some amount of time; this can be achieved by
        yielding the time (in seconds) that should be slept for.
        
        If the step's execution is cancelled, an AbortStep exception will
        be raised at one of the yield points. Regardless of how this exception
        is handled, the step will not be re-entered. It's OK to re-raise
        (e.g., not catch) or to swallow the AbortStep exception."""
        raise NotImplementedError()

    def est_dt(self):
        """Estimate the time required by this step (s)."""
        return 0.0

    def info(self):
        """Describe what this step will do."""
        raise NotImplementedError()

    def json_data(self):
        """Return JSON data describing this step.

        Should be a dictionary containing the 'type' key (giving the step
        type), and any other relevant data."""
        raise NotImplementedError()


# Various useful Steps
default_steps = {}
def default_step(cls):
    default_steps[cls.__name__] = cls
    return cls

@default_step
class ScopeOnStep(Step):
    """Set the scope pin high (tell ThorImage to start recording)."""
    def run(self, olf):
        olf.ctrl.scope_high()
    
    def info(self):
        return 'Start recording'
    
    def json_data(self):
        return {'type': 'scope-on'}

@default_step
class ScopeOffStep(Step):
    """Set the scope pin low (tell ThorImage to stop recording)."""
    def run(self, olf):
        olf.ctrl.scope_low()
        
    def info(self):
        return 'Stop recording'
    
    def json_data(self):
        return {'type': 'scope-off'}

@default_step
class MirrorUpStep(Step):
    """Ensure that the flipper mirror is raised.
    Wait 0.6s for it to move into position if we have to move it."""

    def run(self, olf):
        if olf.mirror.raise_mirror():
            yield 0.6

    def est_dt(self):
        return 0.85
    
    def info(self):
        return 'Raise the flipper mirror'
    
    def json_data(self):
        return {'type': 'mirror-up'}

@default_step
class MirrorDownStep(Step):
    """Ensure that the flipper mirror is lowered.
    Wait 0.6s for it to move into position if we have to move it."""

    def run(self, olf):
        if olf.mirror.lower_mirror():
            yield 0.6

    def est_dt(self):
        return 0.85
    
    def info(self):
        return 'Lower the flipper mirror'
    
    def json_data(self):
        return {'type': 'mirror-down'}

@default_step
class MirrorFlipStep(Step):
    """Flip the flipper mirror."""

    def run(self, olf):
        olf.mirror.flip_mirror()
        yield 0.6
    
    def est_dt(self):
        return 0.6

    def info(self):
        return 'Flip the flipper mirror'
    
    def json_data(self):
        return {'type': 'mirror-flip'}

@default_step
class WaitStep(Step):
    """Pause for some amount of time."""

    def __init__(self, dt):
        self.dt = dt
    
    def run(self, olf):
        yield self.dt
    
    def est_dt(self):
        return self.dt
    
    def info(self):
        return f'Wait {self.dt} s'
    
    def json_data(self):
        return {'type': 'wait', 'time': self.dt}

@default_step
class OdorStep(Step):
    """Deliver an odor."""

    def __init__(self, odor, dt):
        self.odor = odor
        self.dt = dt
    
    def run(self, olf):
        olf.ctrl.olfled_high()
        olf.flow.start_odor_delivery([self.odor])
        try:
            yield self.dt
        finally:
            olf.flow.stop_odor_delivery()
            olf.ctrl.olfled_low()
    
    def est_dt(self):
        return self.dt

    def info(self):
        return f'Deliver odor: {self.odor}'

    def json_data(self):
        return {'type': 'stim', 'odor': str(self.odor), 'time': self.dt}

@default_step
class AdjustOdorFlowStep(Step):
    """Adjust MFC flow rates relating to a specific odor."""

    def __init__(self, odor, carrier_rate, odor_rate):
        self.odor = odor
        self.carrier_rate = carrier_rate
        self.odor_rate = odor_rate
    
    def run(self, olf):
        mfcs = [olf.flow.carrier_mfc]
        mfcs += [fp.mfc if fp is not None else None
            for fp in olf.flow.get_odor_paths(self.odor)]
        targets = [self.carrier_rate, self.odor_rate, self.odor_rate]

        assert len(mfcs) == len(targets)
        targets = [t for i,t in enumerate(targets) if mfcs[i] is not None]
        mfcs = [m for m in mfcs if m is not None]
        assert len(mfcs) == len(targets)
        
        for m, t in zip(mfcs, targets):
            m.set_flow_rate(t)

        delta = 1.0
        for m, t in zip(mfcs, targets):
            while abs((x:=m.get_real_rate()) - t) > delta:
                print(f'target: {t}, rate: {x}')
                yield 0.1
    
    def est_dt(self):
        return 2.0
    
    def info(self):
        return f'Set flow to {self.odor_rate}/{self.carrier_rate} for {self.odor}'
    
    def json_data(self):
        return {
            'type': 'odor-flow',
            'odor': str(self.odor),
            'carrier-flow': self.carrier_rate,
            'odor-flow': self.odor_rate
        }
