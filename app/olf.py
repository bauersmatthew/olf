"""This module contains the core application code."""

from types import DynamicClassAttribute
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import QMainWindow
from main_ui import Ui_MainWindow

import os
import sys
import traceback
import time
import re
import math
import threading
import inspect
import copy
import datetime
import random

import yaml

import config
import steps
from errors import *

def files_in_dir(dir, ext):
    """Iterate over all files with extension `ext` in `dir`."""
    assert os.path.isdir(dir)
    for path in os.listdir(dir):
        fullpath = os.path.join(dir, path)
        if os.path.isfile(fullpath) and fullpath.split('.')[-1].lower() == ext:
            yield fullpath

class Version:
    """A version number; can be for the experiment protocol, or for the arduino
    communication protocol.

    A version here consists of a major version and a minor version. Increases in
    the major version signal changes that are not backwards compatible, while
    increases in minor version are backwards compatible. Both version numbers
    have to be non-negative integers less than 256."""

    def __init__(self, maj, min):
        """Initialize from the major (`maj`) and minor (`min`) version numbers."""
        if maj < 1 or maj > 255:
            raise ValueError('invalid major version')
        if min < 1 or min > 255:
            raise ValueError('invalid minor version')
        self.maj = maj
        self.min = min

    @staticmethod
    def from_string(s):
        """Load from a string of the format 'maj.min', where `maj` and `min` are in decimal."""
        try:
            p = s.split('.')
            if len(p) != 2:
                raise ParseError('string must have two parts')
            maj = int(p[0])
            min = int(p[1])
            return Version(maj, min)
        except Exception as exc:
            raise ParseError(f"failed to parse version string: '{s}'") from exc

    def supports(self, other):
        """Whether software running this protocol version supports configurations written for
        the `other` version."""
        return self.maj == other.maj and self.min >= other.min

    def __str__(self):
        """Print as 'maj.min'."""
        return f'{self.maj}.{self.min}'

class Odor:
    """Represents an odor stimulus."""

    def __init__(self, name, conc):
        self.name = name
        self.conc = conc

    def conc_num(self):
        """Get a float value for the concentration."""
        if self.conc is None:
            return 0.0
        if self.conc[-1] == '%':
            return math.log10(float(self.conc[:-1])/100)
        return float(self.conc)

    parse = re.compile(r'^([^(]+) +\(([^)]+)\)$')

    @staticmethod
    def from_string(s):
        """Parse a string of the form 'odor name (concentration)'."""
        try:
            s = s.strip()
            if '(' not in s:
                return Odor(s, None)
            m = Odor.parse.fullmatch(s.strip())
            if m is None:
                raise ParseError('no match')
            g = m.groups()
            if len(g) != 2:
                raise ParseError('invalid # of matched groups')
            return Odor(*g)
        except Exception:
            raise ParseError(f"failed to parse odor: '{s}'")

    def __str__(self):
        if self.conc is None:
            return self.name
        return f'{self.name} ({self.conc})'
    
    def __repr__(self):
        return f'Odor({self.name!r}, {self.conc!r})'

    def __hash__(self):
        return hash(str(self))

    def __eq__(self, other):
        return str(self) == str(other)

class Controller:
    """Parent class to communicate with the olfactometer controller.

    Children only need to implement write() and read(), which are used
    to send/recieve raw bytes from the hardware controller. The interpretation
    of said bytes is implemented here."""
    VERSION = Version(1,1)

    def write(self, b):
        """Write bytes `b` to the olfactometer controller."""
        raise NotImplementedError()

    def read(self, n):
        """Read `n` bytes from the olfactometer controller."""
        raise NotImplementedError()

    def close(self):
        """Close the connection."""
        pass
    
    def wb(self, i): self.write(bytes([i]))
    def wh(self, i): self.wb(128+i)
    def wl(self, i): self.wb(i)

    def reset(self): self.wb(123)
    def mirror_high(self): self.wh(124)
    def mirror_low(self): self.wl(124)
    def scope_high(self): self.wh(127)
    def scope_low(self): self.wl(127)
    def solenoid_high(self, num): self.wh(num)
    def solenoid_low(self, num): self.wl(num)
    def olfled_high(self): self.wh(126)
    def olfled_low(self): self.wl(126)
    def flow_high(self): self.wh(125)
    def flow_low(self): self.wl(125)

    def check_version(self):
        """Check version compatibility with the controller."""
        self.wb(122)
        r = self.read(2)
        if len(r) != 2:
            raise ControllerError('Timeout waiting for controller version.')
        ver = Version(r[0], r[1])
        if not self.VERSION.supports(ver):
            raise VersionError(self.VERSION, ver, 'controller')

class StdioController(Controller):
    """Controller connected to standard I/O for debugging."""
    def write(self, b):
        sys.stdout.write(f'{b[0]}\n')

    def read(self, n):
        line = sys.stdin.readline().strip()
        ints = [int(i) for i in line.split()[:n]]
        return bytes(ints)

class SerialController(Controller):
    """Connect over serial/COM port."""
    def __init__(self, port):
        """Open the serial connection."""
        try:
            import serial
            s = serial.Serial(port, 9600, timeout=1)
            time.sleep(1.5) # need to give serial time to actually connect
            self.serial = s
        except Exception as exc:
            raise ControllerError(f'Failed to start serial on {port}')

    def write(self, b):
        try:
            self.serial.write(b)
        except Exception as exc:
            raise ControllerError('Failed to write to serial!')
    
    def read(self, n):
        try:
            return self.serial.read(n)
        except Exception as exc:
            raise ControllerError('Failed to read from serial!')

    def close(self):
        self.serial.close()

class StdioFlowController:
    """Mock flow controller that writes to stdout."""
    def __init__(self, name):
        self.name = name
        self.rate = 0.0

    def set_flow_rate(self, rate):
        sys.stdout.write(f'[FC: {self.name}] SET FLOW RATE: {rate}\n')
        self.rate = rate
    
    def get_target_rate(self):
        return self.rate
    
    def get_real_rate(self):
        return self.rate*(0.9 + random.random()*0.2)
    
    def close(self):
        pass

class FlowController:
    """Real flow controller that communicates over serial.

    Essentially just wraps `alicat.FlowController`, but using this class instead
    helps control (restrict) which methods are used; in particular this lets
    us ensure that `FlowController` and `StdioFlowController` have the same
    interface.

    Additionally, using this class makes it easier to control whether `alicat`
    is imported.
    """

    def __init__(self, port):
        import alicat
        self.c = alicat.FlowController(port=port)
        # The FlowController() constructor for some reason fails silently if
        # it can't connect to the MFC. However, if the connection is invalid,
        # then the .get() function will raise an exception. Therefore we call
        # .get() here just to check whether the connection is actually valid.
        try:
            self.c.get()
        except:
            raise Error('failed to connect!')
    
    def set_flow_rate(self, rate):
        try:
            self.c.set_flow_rate(rate)
        except Exception as exc:
            # Sometimes alicat.FlowController.set_flow_rate will complain that
            # it can't set the setpoint, even when it appears to succeed
            # (inspecting both the results of .get() and the setpoint on the
            # physical MFC). From my point of view, this exception is a bug.
            # Therefore, we check whether we _actually_ failed to set the
            # setpoint by reading back the setpoint from the physical MFC,
            # and checking whether it matches the desired value.
            if str(exc) == 'Could not set setpoint.':
                # If .get() fails, then there is definitely something wrong,
                # so it's OK to just raise another exception.
                real_target = self.get_target_rate()
                if abs(real_target - rate) > 0.1:
                    # The 
                    raise
    
    def get_target_rate(self):
        return self.c.get()['setpoint']

    def get_real_rate(self):
        return self.c.get()['volumetric_flow']
    
    def close(self):
        self.c.close()

class FlowPath(QObject):
    """Controls one gas flow path."""

    ROUTE_DNE = -1
    ROUTE_IS_DEFAULT = -2

    class FlowPathRoute:
        """Passed to Qt slots/signals."""
        def __init__(self, a, b):
            self.name = a
            self.pin = b
    route_changed = pyqtSignal(FlowPathRoute)
    rate_changed = pyqtSignal(float)

    def __init__(self, name,
            solcon: Controller, mfc: FlowController,
            balance_pin, vent_pin, odor_pins):
        super().__init__()
        self.name = name
        self.mfc = mfc
        self.solcon = solcon
        self.balance_pin = balance_pin
        self.vent_pin = vent_pin
        self.odor_pins = odor_pins

        assert vent_pin != balance_pin
        if vent_pin == self.ROUTE_IS_DEFAULT:
            assert balance_pin != self.ROUTE_IS_DEFAULT
        assert self.can_balance() or self.can_vent()

        used_pins = set([vent_pin, balance_pin])
        for pin in odor_pins:
            assert pin not in used_pins
            assert pin >= 0
            used_pins.add(pin)

    def set_flow_rate(self, rate):
        """Set the flow rate."""
        self.mfc.set_flow_rate(rate)
        self.rate_changed.emit(rate)
    
    def close_odor_solenoids(self, *, leave=None):
        """Close all odor solenoids except the one specified by `leave`."""
        for pin in self.odor_pins:
            if pin != leave:
                self.solcon.solenoid_low(pin)
    
    def close_nondefault_vent(self):
        """Ensure that the (non-default) vent route is closed."""
        if self.vent_pin >= 0:
            self.solcon.solenoid_low(self.vent_pin)
    
    def close_nondefault_balance(self):
        """Ensure that the balance route (if available) is closed."""
        if self.balance_pin >= 0:
            self.solcon.solenoid_high(self.balance_pin)

    def direct_to_odor(self, pin):
        """Direct flow through an odor route."""
        self.solcon.solenoid_high(pin)
        self.close_nondefault_balance()
        self.close_nondefault_vent()
        self.close_odor_solenoids(leave=pin)
        self.route_changed.emit(self.FlowPathRoute('odor', pin))
    
    def direct_to_vent(self):
        """Direct flow through the vent route."""
        if self.vent_pin == self.ROUTE_DNE:
            raise Error(f'Flow route {self.name} does not support venting!')
        elif self.vent_pin >= 0:
            self.solcon.solenoid_high(self.vent_pin)
        else: assert self.vent_pin == self.ROUTE_IS_DEFAULT
        self.close_odor_solenoids()
        self.close_nondefault_balance()
        self.route_changed.emit(self.FlowPathRoute('vent', self.vent_pin))
    
    def direct_to_balance(self):
        """Direct flow through the balance route."""
        if self.balance_pin == self.ROUTE_DNE:
            raise Error(f'Flow route {self.name} does not support balancing!')
        elif self.balance_pin >= 0:
            self.solcon.solenoid_low(self.balance_pin)
        else: assert self.balance_pin == self.ROUTE_IS_DEFAULT
        self.close_nondefault_vent()
        self.close_odor_solenoids()
        self.route_changed.emit(
            self.FlowPathRoute('balance', self.balance_pin))
    
    def reset_route(self):
        """Direct flow to wherever it goes when all pins are set low.

        This should be guaranteed (by HARDWARE) to be a safe state. Normally:

        - If can_balance(), then the route changes to balancing.
        - Otherwise, the route changes to venting.

        Configurations disobeying these rules are imaginable, but probably
        shouldn't be constructed. Or if they are, they shouldn't be used with
        this code as-is.

        The signal emitted to `route_changed` is according to these rules,
        even if they do not reflect reality.
        """
        if self.balance_pin >= 0:
            self.solcon.solenoid_low(self.balance_pin)
        if self.vent_pin >= 0:
            self.solcon.solenoid_low(self.vent_pin)
        self.close_odor_solenoids()
        
        if self.can_balance():
            self.route_changed.emit(
                self.FlowPathRoute('balance', self.balance_pin))
        else:
            # ==> can_vent()
            self.route_changed.emit(
                self.FlowPathRoute('vent', self.vent_pin))
    
    def can_vent(self):
        """Whether the route can direct air away from the fly."""
        return self.vent_pin != self.ROUTE_DNE
    
    def can_balance(self):
        """Whether the route can direct plain air at the fly."""
        return self.balance_pin != self.ROUTE_DNE
    
    def has_mfc(self):
        """Whether an MFC is available to control mass flow."""
        return self.mfc is not None

class FlowManager:
    """Manage airflow throughout the olfactometer."""

    def __init__(self, solcon):
        """Initialze mostly empty, but connected to a `sol`enoid `con`troller."""
        self.solcon = solcon
        self.carrier_mfc = None
        self.mfcs = {}
        self.paths = {}
        self.balance_proxies = {}
        self.pin2path = {}
        self.odor2pin = {}
        self.pin2odor = {}
    
    def open_mfc_connection(self, port):
        """Open a connection to the MFC at `port`.

        Raises an Error if this connection is already open.
        """
        if port in self.mfcs:
            raise Error(f"A connection to the MFC at {port} is already open!")
        try:
            if port.startswith('debug'):
                self.mfcs[port] = StdioFlowController(port)
            else:
                self.mfcs[port] = FlowController(port)
        except:
            raise Error(f'Failed to connect to MFC at {port}!')
        return self.mfcs[port]

    def connect_carrier_mfc(self, port):
        """Connect & register the carrier MFC at `port`."""
        self.carrier_mfc = self.open_mfc_connection(port)
    
    def add_path(self, path: FlowPath, proxy: str):
        """Register FlowPath `path`, with `proxy` as its balance proxy.

        The balance proxy is used to provide balancing air flow when `path`
        is vented; this is useful in when the flow block/manifold corresponding
        to `path` is unable to balance its own flow.

        Here, `proxy` is specified by name. If a path with such a name has not
        been loaded, an Error is raised. If the path exists but is not a
        valid balance proxy (because it cannot vent, or cannot balance), an
        Error is raised.
        """
        # Validate balance proxy.
        if proxy is not None:
            if proxy not in self.paths:
                raise Error(f'Unrecognized balance proxy: {proxy!r}')
            try:
                theproxy = self.paths[proxy]
                assert theproxy.can_balance()
                assert theproxy.can_vent()
            except:
                raise Error(f'Invalid balance proxy: {proxy!r}')
        else:
            assert path.can_balance()
        
        # Validate stim path.
        if path.name in self.paths:
            raise Error(f'Reused path name: {path.name!r}')
        for pin in path.odor_pins:
            if pin in self.pin2path:
                raise Error(f'Pin {pin!r} belongs to multiple paths!')

        # Register.
        self.paths[path.name] = path
        self.balance_proxies[path.name] = proxy
        for pin in path.odor_pins:
            self.pin2path[pin] = path.name
    
    def add_odor(self, odor: Odor, pin: int):
        """Register `odor` as being addressed by `pin`.

        Raise an Error if either has been mapped before.
        """
        if odor in self.odor2pin:
            raise Error(f'Remapped odor: {odor!r}')
        if pin in self.pin2odor:
            raise Error(f'Remapped pin: {pin!r}')
        self.odor2pin[odor] = pin
        self.pin2odor[pin] = odor
    
    def has_odor_mapping(self):
        """Return whether any odors are mapped to pins."""
        return len(self.odor2pin) > 0
    
    def start_odor_delivery(self, odors):
        """Attempt to simultaneously deliver all given odors.

        Delivery continues until `stop_odor_delivery()` is called.
        """
        pins = [self.odor2pin[o] for o in odors]
        path_names = [self.pin2path[pin] for pin in pins]

        # Check that each path is used at most once
        if len(set(path_names)) != len(pins):
            raise Error(
                'Cannot simultaneously deliver two odors from the same path!')
        for name in path_names:
            proxy_name = self.balance_proxies[name]
            if proxy_name in path_names:
                raise Error(
                    'Cannot deliver odors from path {name!r} '
                    'and its balance proxy {proxy_name!r} simultaneously!'
                )
        
        # Deliver
        for name, pin in zip(path_names, pins):
            path: FlowPath = self.paths[name]
            path.direct_to_odor(pin)
            proxy_name = self.balance_proxies[name]
            if proxy_name is not None:
                proxy: FlowPath = self.paths[proxy_name]
                proxy.direct_to_vent()
    
    def stop_odor_delivery(self):
        """Reset all flow paths to their default (zeroed) states.

        Use to stop an odor presentation started by `start_odor_delivery()`.
        """
        for path in self.paths.values():
            path.reset_route()

    def get_odor_paths(self, odor: Odor):
        """Get the FlowPaths that control delivery of `odor`.

        Returns a tuple `(stim_path, proxy_path)` where `stim_path` is the path
        used for odor delivery, and `proxy_path` is used for balancing.
        """
        pin = self.odor2pin[odor]
        stim_path_name = self.pin2path[pin]
        stim_path = self.paths[stim_path_name]
        proxy_path_name = self.balance_proxies[stim_path_name]
        proxy_path = None
        if proxy_path_name is not None:
            proxy_path = self.paths[proxy_path_name]
        return stim_path, proxy_path
    
    def close(self):
        """Close all MFC connections."""
        for mfc in self.mfcs.values():
            mfc.close()
    
class MirrorManager(QObject):
    """Manage the flipper mirror state."""
    
    # Emitted when the registered state changes; this could be due to internal
    # instructions (e.g., when the mirror is flipped) or external instructions
    # (e.g., when the user tells the program what state the mirror is in).
    state_changed = pyqtSignal(bool)

    def __init__(self, ctrl):
        """Initialize assuming that the flipper mirror is blocking."""
        super().__init__()
        self.ctrl = ctrl
        self.blocking = False
    
    def flip_mirror(self):
        """Flip the mirror position."""
        self.ctrl.mirror_high()
        time.sleep(0.25)
        self.ctrl.mirror_low()
        self.set(not self.blocking)
    
    def raise_mirror(self):
        """Ensure that the mirror is in the raised (blocking) position.

        Return whether the position was changed.
        """
        if not self.blocking:
            self.flip_mirror()
            return True
        return False
    
    def lower_mirror(self):
        """Ensure that the mirror is in the lowered (nonblocking) position.

        Return whether the position was changed.
        """
        if self.blocking:
            self.flip_mirror()
            return True
        return False
    
    def set(self, value):
        """Manually set the currently recorded mirror state."""
        if value != self.blocking:
            self.blocking = value
            self.state_changed.emit(self.blocking)

class Olfactometer:
    """Loads and holds connections to all the hardware that we control."""

    def __init__(self, ctrl, flow, mirror, source):
        self.ctrl = ctrl
        self.mirror = mirror
        self.flow = flow
        self.source = source
        self.odors = list(self.flow.odor2pin.keys())

    @classmethod
    def from_files(cls, hardware_path, experiment_path):
        """Load from the configs at `hardware_path` and `experiment_path`.

        The hardware config describes the (mostly static) hardware
        setup of the olfactometer: in particular it describes the available
        flow paths and their associated solenoids, as well as how to connect
        to all the various controllers (MFCs and the arduino).

        The experiment config describes
        (1) how odors have been loaded into the olfactometer, and
        (2) what parameters should be passed to plan generators.

        Returns `(Olfactometer, plan_cfgs)` where `plan_cfgs` is a dict
        containing configuration info for plan generation.
        """
        try:
            pin_map, plan_cfgs = cls.load_experiment_config(experiment_path)
        except:
            raise Error('failed to load experiment config')
        try:
            ctrl, flow, mirror = cls.load_hardware_config(
                hardware_path, pin_map)
        except:
            raise Error('failed to load hardware config')
        return (
            cls(ctrl, flow, mirror, (hardware_path, experiment_path)),
            plan_cfgs
        )
    
    @staticmethod
    def load_experiment_config(path):
        """Load an experiment config file.

        See `from_files()`. Returns `(pinmap, plancfgs)` where `pinmap`
        is a dict Odor --> pin and `plancfgs` is a JSON-like dict containing
        config settings for plan generation.
        """
        with open(path) as fin:
            data = yaml.safe_load(fin)
        if not isinstance(data, dict):
            raise Error(f'top level must be dict, not {type(data)}!')
        
        if 'odors' not in data:
            raise Error('no pin-odor map specified!')
        pins_raw = data['odors']
        if not isinstance(pins_raw, dict):
            raise Error("'odors' field must provide a dict")
        pinmap = {}
        for k, v in pins_raw.items():
            if not isinstance(k, int):
                raise Error('invalid pin (not an int)')
            try:
                o = Odor.from_string(v)
            except:
                raise Error('invalid odor for pin: {k}')
            pinmap[o] = k
        
        plancfgs = {}
        for k, v in data.items():
            if k == 'odors': continue
            if not isinstance(v, dict):
                raise Error('plan config for {k} is not a dict!')
            plancfgs[k] = v
        
        return pinmap, plancfgs
    
    @staticmethod
    def load_hardware_config(path, pinmap):
        """Load a hardware config file, and setup the described hardware.

        See `from_files()`.
        Returns `(Controller, FlowManager, MirrorManager)`.
        """
        with open(path) as fin:
            data = yaml.safe_load(fin)
        if not isinstance(data, dict):
            raise Error(f'top level must be dict, not {type(data)}!')
        
        ctrl = None
        flow = None
        try:
            # Load arduino controller
            if 'controller' not in data:
                raise Error('no controller port specified!')
            ctrl_port = data['controller']
            try:
                if ctrl_port == 'debug':
                    ctrl = StdioController()
                else:
                    ctrl = SerialController(ctrl_port)
            except:
                raise Error('failed to connect to controller')
            mirror = MirrorManager(ctrl)
        
            # Load flow manager
            flow = FlowManager(ctrl)
            if 'carrier-mfc' in data:
                carrier_port = data['carrier-mfc']
                if carrier_port is not None:
                    flow.connect_carrier_mfc(carrier_port)
            
            ## Load flow paths
            if 'flows' not in data:
                raise Error('no flows specified!')
            flows_data = data['flows']
            if not isinstance(flows_data, dict):
                raise Error("'flows' must be a dict!")
            for name, dflow in flows_data.items():
                try:
                    Olfactometer.load_flow_path(ctrl, flow, name, dflow)
                except:
                    raise Error(f'failed to load flow path {name!r}')
                
            # Register odor->pin mapping
            for odor, pin in pinmap.items():
                flow.add_odor(odor, pin)
        
            return ctrl, flow, mirror
        except:
            if flow is not None:
                flow.close()
            if ctrl is not None:
                ctrl.close()
            raise
    
    @staticmethod
    def load_flow_path(ctrl, flow, name, dflow):
        """Helper function for `load_hardware_config()`."""
        mfc = None
        if 'mfc' in dflow:
            mfc_port = dflow['mfc']
            if mfc_port is not None:
                mfc = flow.open_mfc_connection(mfc_port)
        
        if 'pins' not in dflow:
            raise Error('pins not configured!')
        dpins = dflow['pins']
        if not isinstance(dpins, dict):
            raise Error("'pins' must be a dict!")
        
        # Parse balance parameter.
        # There must be some way to balance the flow! (Can't accept none)
        balance_param = dpins['balance']
        proxy_name = None
        if isinstance(balance_param, int):
            balance_pin = balance_param
        elif balance_param == 'default':
            balance_pin = FlowPath.ROUTE_IS_DEFAULT
        elif (isinstance(balance_param, str)
                and balance_param.startswith('=')):
            balance_pin = FlowPath.ROUTE_DNE
            proxy_name = balance_param[1:]
        else:
            raise Error(f'bad balance param: {balance_param!r}')
        
        # Parse vent parameter.
        vent_param = dpins['vent']
        if isinstance(vent_param, int):
            vent_pin = vent_param
        elif vent_param in ('none', None):
            vent_pin = FlowPath.ROUTE_DNE
        elif vent_param == 'default':
            vent_pin = FlowPath.ROUTE_IS_DEFAULT
        else:
            raise Error(f'Invalid vent parameter: {vent_param!r}')
        
        # Construct & register.
        stim_pins = dpins['odor'] # will be caught by FlowPath if invalid
        path = FlowPath(name, ctrl, mfc, balance_pin, vent_pin, stim_pins)
        flow.add_path(path, proxy_name)

    def close(self):
        """`close()` all relevant sub-managers."""
        self.flow.close()
        self.ctrl.close()

class Plan:
    """Represent a non-stateful experiment plan."""
    def __init__(self, name, params, steplist):
        self.name = name
        self.steps = steplist.serialize()
        self.steplist = steplist
        self.params = params

    def est_dt(self):
        """Estimate the time required to run the entire plan."""
        return sum([s.est_dt() for s in self.steps])

    def info(self):
        """Get info for each step."""
        return [s.info() for s in self.steps]

    def json_data(self):
        return {
            'name': self.name,
            'config': self.params,
            'steps': [s.json_data() for s in self.steps]
        }

class PlanLoader:
    """Manage loaded plans and plan generators."""

    def __init__(self):
        self.plans = {}
        self.plugin_env = {
            'plan': self._plan_deco,
            'Step': steps.Step,
            'StepList': steps.StepList
        }
        self.plugin_env.update(steps.default_steps)
        self.configs = {}
    
    def _plan_deco(self, name, config_tag):
        """Registration decorator for plan generation functions.

        `name` is the pretty-printing plan name. `config_tag` is the key for
        the plan's sub-config in experiment config files.
        """
        def _plan_inner_1(fun):
            def _plan_inner_2(odors, config):
                l = fun(odors, config)
                if not isinstance(l, steps.StepList):
                    l = steps.StepList.from_list(l)
                return Plan(name, config, l)
            self.plans[name] = (_plan_inner_2, config_tag)
            return _plan_inner_2
        return _plan_inner_1
    
    def load_plan_plugin(self, path):
        """Load one plan plugin .py file."""
        with open(path) as fin:
            src = fin.read()
        exec(src, self.plugin_env)

    def generate_plan(self, plan_name, odors):
        """Generate one Plan from the plan generator named `plan_name`."""
        genfun, cfgtag = self.plans[plan_name]
        cfg = {}
        if cfgtag in self.configs:
            cfg = self.configs[cfgtag]
        return genfun(odors, cfg)

class RunState:
    """Contains information about the state of a run.

    Intended to be used as a payload for Qt slots/signals."""
    def __init__(self, plan, step, status, timing):
        self.plan = plan
        self.step = step
        self.status = status
        self.timing = timing

class RunExitStatus:
    """Contains information about the exit status of a run."""
    def __init__(self,
            time_start, time_stop,
            plan, last_step, aborted, last_step_aborted,
            exit_ok, exc_info):
        self.time_start = time_start
        self.time_stop = time_stop
        self.plan = plan
        self.aborted = aborted
        self.last_step_aborted = last_step_aborted
        self.last_step = last_step
        self.exit_ok = exit_ok
        self.exc_info = exc_info

class PlanRunner(QObject):
    "Handles execution of Plans."""

    class RunUpdate:
        """Helper class to generate PlanState instances with timing info."""

        def __init__(self, plan, step_idx, t0_run, t0_step):
            self.plan = plan
            self.step = step_idx
            self.t0_run = t0_run
            self.t0_step = t0_step

        def gen_status(self):
            """Generate a filled PlanState instance."""
            t = time.time()
            step_dt = self.plan.steps[self.step].est_dt()
            step_remain = self.t0_step+step_dt-t
            total_remain = (
                + sum([s.est_dt() for s in self.plan.steps[self.step+1:]])
                + step_remain
            )
            step_prog = 1.0
            total_prog = 1.0
            if step_dt > 0:
                step_prog = max(0.0, 1 - step_remain/step_dt)
            if (plan_dt := self.plan.est_dt()) > 0:
                total_prog = max(0.0, 1 - total_remain/plan_dt)
            timing = (step_prog, step_remain, total_prog, total_remain)
            return RunState(self.plan, self.step, 'running', timing)

    update = pyqtSignal(RunState)
    stopped = pyqtSignal(RunExitStatus)

    def __init__(self, olf, plan):
        super().__init__()

        self.olf = olf   # olfactometer
        self.plan = plan # plan to run

        self.wait_event = threading.Event()
        
        self.aborted = False # whether execution deviated nicely from planned
        self.last_step_aborted = False

        self.last_step = None
        self.exit_ok = True    # whether no unhandled exceptions were thrown
        self.exc_info = None   # sys.exc_info(), if relevant

        self.running = False
    
    def run(self):
        try:
            self.running = True
            t0 = time.time()
            for i, s in enumerate(self.plan.steps):
                ru = self.RunUpdate(self.plan, i, t0, time.time())
                self.send_update(ru)
                self.last_step = i
                if not inspect.isgeneratorfunction(s.run):
                    # The step is 'atomic' and can't be cancelled.
                    s.run(self.olf)
                else:
                    # The step can be cancelled, and may include waits.
                    g = s.run(self.olf)
                    while True:
                        # Execute each sub-step.
                        try:
                            x = g.__next__()
                        except StopIteration: break # (step finished)
                        if x is not None:
                            # Step used 'yield [time]', instructing us to
                            # wait for [time] seconds before re-entering.
                            self.wait(x, ru)
                        self.send_update(ru)
                        if self.wait_event.is_set():
                            # abort() has been called! Tell the step to clean
                            # up and exit nicely by throwing AbortStep() at
                            # the `yield` line.
                            self.last_step_aborted = True
                            try:
                                g.throw(steps.AbortStep())
                            except steps.AbortStep: pass
                if self.wait_event.is_set():
                    break
            if self.last_step < len(self.plan.steps)-1 or self.last_step_aborted:
                # We use this instead of the state of self.wait_event because
                # it better reflects whether any deviation from the normal
                # execution plan was actually made.
                self.aborted = True
        except:
            self.exit_ok = False
            self.exc_info = sys.exc_info()
        finally:
            self.running = False
            self.time_start = datetime.datetime.fromtimestamp(
                t0).isoformat(timespec='seconds')
            self.time_stop = datetime.datetime.fromtimestamp(
                time.time()).isoformat(timespec='seconds')
            self.stopped.emit(self.exit_state())

    def wait(self, dt, ru):
        """Wait for dt seconds, sending updates, in an interruptable way."""
        # This wait is implemented by waiting for wait_event to be set
        # with a timeout of dt; that way we can be interrupted with no delay.
        t0 = time.time()
        while time.time()-t0 < dt-0.11:
            self.send_update(ru)
            if self.wait_event.wait(0.1):
                break
        # Event.wait can handle negative timeouts
        self.wait_event.wait(t0+dt-time.time())
    
    def send_update(self, ru):
        """Emit an update about the current run status."""
        self.update.emit(ru.gen_status())

    def abort(self):
        """Abort plan execution, as cleanly as possible."""
        self.wait_event.set()

    def exit_state(self):
        """Get an RunExitState object about the exit state of this run."""
        if self.running:
            raise RuntimeError('Run is still running!')
        return RunExitStatus(
            self.time_start, self.time_stop,
            self.plan, self.last_step, self.aborted, self.last_step_aborted,
            self.exit_ok, self.exc_info)

class ReviewInfo:
    """Holds review information about previous runs."""

    def __init__(self, exit_status):
        if exit_status is None: return
        self.status_data = {
            'start-time': exit_status.time_start,
            'stop-time': exit_status.time_stop,
            'aborted': exit_status.aborted,
            'exit-ok': exit_status.exit_ok,
            'plan': exit_status.plan.json_data()
        }
        last = exit_status.last_step
        for i, d in enumerate(self.status_data['plan']['steps']):
            if i < last:
                x = 'run'
            elif i > last:
                x = 'not-run'
            else:
                if exit_status.last_step_aborted: x = 'aborted'
                else: x = 'run'
            assert 'run-status' not in d
            d['run-status'] = x
        self.extras_raw = ''
        self.extras_format = 'YAML'
        self.notes = ''
    
    def json_data(self):
        d = copy.copy(self.status_data)

        ex = self.extras_raw.strip()
        if len(ex) > 0:
            f = self.extras_format
            if f == 'YAML':
                import yaml
                d |= yaml.safe_load(ex)
            else: #json
                import json
                d |= json.loads(ex)
        
        notes = self.notes.strip()
        if len(notes) > 0:
            d['notes'] = notes
        
        return d

class RunManager(QObject):
    """Keeps track of active runs and miniruns."""
    
    run_started = pyqtSignal(Plan)
    run_updated = pyqtSignal(RunState)
    run_stopped = pyqtSignal(RunExitStatus)
    minirun_started = pyqtSignal(Plan)
    minirun_updated = pyqtSignal(RunState)
    minirun_stopped = pyqtSignal(RunExitStatus)

    def begin(self):
        """Start threads."""
        self.run = None
        self.run_thread = QtCore.QThread()
        self.run_thread.start()

        self.minirun = None
        self.minirun_thread = QtCore.QThread()
        self.minirun_thread.start()
    
    def idle(self):
        """True if no run or minirun is active."""
        return self.run is None and self.minirun is None

    def start_run(self, olf, plan):
        """Start and connect a PlanRunner for the main plan."""
        assert self.idle()
        self._start_any(olf, plan, 'run')
    
    def start_minirun(self, olf, miniplan):
        """Start and connect a PlanRunner for a mini-plan."""
        assert self.minirun is None
        self._start_any(olf, miniplan, 'minirun')
    
    def _start_any(self, olf, plan, which):
        """Helper for start_run() and start_minirun()."""
        pr = PlanRunner(olf, plan)
        pr.moveToThread(self.__getattribute__(f'{which}_thread'))
        self.__getattribute__(f'{which}_started').connect(pr.run)
        pr.update.connect(self.__getattribute__(f'on_{which}_update'))
        pr.stopped.connect(self.__getattribute__(f'on_{which}_exit'))
        self.__setattr__(which, pr)
        self.__getattribute__(f'{which}_started').emit(plan)
    
    def abort_run(self):
        """Abort the main plan runner (raise if no run is in progress)."""
        assert self.run is not None
        self.run.abort()
    
    def abort_minirun(self, olf, plan):
        """Abort the miniplan runner (raise if no minirun is in progress)."""
        assert self.minirun is not None
        self.minirun.abort()
    
    def on_run_update(self, rs: RunState):
        """Forward run update."""
        self.run_updated.emit(rs)
    
    def on_minirun_update(self, rs):
        """Forward minirun update."""
        self.minirun_updated.emit(rs)
    
    def on_run_exit(self, es: RunExitStatus):
        """Forward run exit status."""
        self.run = None
        self.run_stopped.emit(es)
    
    def on_minirun_exit(self, es):
        """Forward minirun exit status."""
        self.minirun = None
        self.minirun_stopped.emit(es)

    def quit(self):
        """Quit all threads, aborting runs if necessary."""
        if self.run is not None: self.run.abort()
        if self.minirun is not None: self.minirun.abort()
        self.run_thread.quit()
        self.minirun_thread.quit()
        self.run_thread.wait()
        self.minirun_thread.wait()

class Worker(QObject):

    # Emitted when the worker is fully reset, and everything is unloaded.
    state_reset = pyqtSignal()

    # Emitted when a new Olfactometer is loaded.
    reconfigured = pyqtSignal(Olfactometer, PlanLoader)
    
    # Emitted when a new main Plan is loaded.
    plan_generated = pyqtSignal(Plan)

    # Emitted when an error is encountered.
    # The first argument is a nice description of the error.
    # The second argument is the contents of sys.exc_info(), which will be
    # (None, None, None) in the case of no exception.
    error_encountered = pyqtSignal(str, tuple)

    # Emitted when an unrecoverable error is encountered.
    # The argumet is the contents of sys.exc_info(), which will always have
    # an exception.
    panic_encountered = pyqtSignal(tuple)

    def __init__(self):
        """Empty initialization; see `begin()`.

        The `run_manager` attribue is required so that the UI can connect to
        its signals.
        """
        super().__init__()
        self.run_manager = RunManager()
    
    def begin(self):
        """Actual initialization happens here.

        This should be called *in the worker thread*, after it's been started.
        This is required to ensure that all the threads will be configured
        correctly.
        """
        self.olf = None
        self.plan_loader = PlanLoader()
        self.plan = None # the plan that's previewed by the UI
        self.run_manager.begin()
        self.has_experiment_config = False

        # Load plan plugins
        for dir in config.plan_dirs:
            for path in files_in_dir(dir, 'py'):
                try:
                    self.plan_loader.load_plan_plugin(path)
                except:
                    self.panic(f'Failed to load plan plugin: {path}')

    def connect_ui(self, ui):
        """Connect the UI's signals to our slots.
        
        This should be called after we have been moved into the worker thread;
        otherwise, slots will be called in the same thread (this is a
        'peculiarity' of PyQt5).
        """
        ui.configs_chosen.connect(self.configure)
        ui.config_reset_requested.connect(self.try_reset)
        ui.plan_chosen.connect(self.generate_plan)
        ui.controller_reset_requested.connect(self.reset_controller)
        ui.start_requested.connect(self.start_run)
        ui.stop_requested.connect(self.stop_run)
        ui.mirror_flip_requested.connect(self.flip_mirror)
        ui.new_mirror_state.connect(self.set_mirror_state)
        ui.odortest_requested.connect(self.run_odortest)

    def assert_inert(self):
        """Emit an error message if any run is in progress."""
        if not self.run_manager.idle():
            self.error.emit('A run is in progress!')
            return False
        return True

    def emit_error(self, msg):
        """Emit an error via the `error_encountered` signal."""
        self.error_encountered.emit(msg, sys.exc_info())
    
    def panic(self, msg=None):
        """Emit via `panic_encountered`."""
        try:
            if msg is not None:
                raise Error(f'PANIC: {msg}')
            raise Error('PANIC!')
        except Exception as exc:
            self.panic_encountered.emit(sys.exc_info())
            return exc
    
    def reset(self):
        """Reset to starting state. Panics if not inert!"""
        if not self.run_manager.idle():
            raise self.panic()
        if self.olf is not None:
            self.olf.close()
            self.olf = None
        self.plan = None
        try:
            if self.run_manager.run is not None:
                self.run_manager.abort_run()
            if self.run_manager.minirun is not None:
                self.run_manager.abort_minirun()
        except:
            raise self.panic()
        self.state_reset.emit()

    def if_inert(fun):
        """Decorator to call `assert_inert()` before proceeding."""
        def _if_inert_inner(self, *args, **kwargs):
            if not self.assert_inert(): return
            return fun(self, *args, **kwargs)
        return _if_inert_inner
    
    def if_configured(fun):
        """Decorator to check configuration before proceeding."""
        def _if_configured_inner(self, *args, **kwargs):
            if self.olf is None:
                self.emit_error('Not configured!')
                return
            return fun(self, *args, **kwargs)
        return _if_configured_inner

    @if_inert
    def try_reset(self):
        """Try to reset the state, if it's safe to do so."""
        self.reset()

    @if_inert
    def configure(self, hardware_path, experiment_path):
        """Load `self.olf` from the given files."""
        try:
            if self.olf is not None:
                self.reset()
            self.olf, cfgs = Olfactometer.from_files(
                hardware_path, experiment_path)
            self.plan_loader.configs = cfgs
            self.reconfigured.emit(self.olf, self.plan_loader)
        except:
            self.olf = None
            self.emit_error('Configuration failed!')
    
    @if_inert
    @if_configured
    def generate_plan(self, name):
        """Generate a new main plan from the given plan name."""
        try:
            self.plan = self.plan_loader.generate_plan(name, self.olf.odors)
        except:
            self.emit_error('Failed to load plan!')
        else:
            self.plan_generated.emit(self.plan)
    
    @if_inert
    @if_configured
    def start_run(self):
        """Start running the main plan."""
        try:
            if self.plan is None:
                return self.emit_error('No plan to run!')
            self.run_manager.start_run(self.olf, self.plan)
        except:
            self.emit_error('Failed to start run!')

    def stop_run(self):
        """Abort the running plan."""
        try:
            self.run_manager.abort_run()
        except:
            self.emit_error('Failed to abort run!')
    
    @if_configured
    def reset_controller(self):
        """Reset the controller."""
        try:
            self.olf.ctrl.reset()
        except:
            self.emit_error('Failed to reset controller!')

    @if_configured
    def flip_mirror(self):
        """Start a miniplan to flip the flipper mirror."""
        try:
            miniplan = Plan(None, None,
                steps.StepList.from_list([steps.MirrorFlipStep()]))
            self.run_manager.start_minirun(self.olf, miniplan)
        except:
            self.emit_error('Failed to start mirror minirun!')

    @if_configured
    def set_mirror_state(self, state):
        """Override the internal state of the flipper mirror."""
        self.olf.mirror.set(state)

    @if_configured
    def run_odortest(self, odor):
        """Start a miniplan to run an odor test."""
        try:
            miniplan = Plan(None, None,
                steps.StepList.from_list([steps.OdorStep(odor, 2.0)]))
            self.run_manager.start_minirun(self.olf, miniplan)
        except:
            self.emit_error('Failed to start odortest minirun!')
    
    def quit(self):
        """Clean up."""
        try:
            if self.olf is not None:
                self.olf.close()
            self.run_manager.quit()
        except:
            raise self.panic()

class UI(QMainWindow):

    # Emitted when the user requests to configure the program. Arguments:
    # 1. The hardware configuration path
    # 2. The experiment configuration path
    configs_chosen = pyqtSignal(str, str)

    config_reset_requested = pyqtSignal()
    
    # Emitted when (1) the user selects a new plan from the combobox, or
    # (2) the user presses the Reload button. Arguments:
    # 1. The (human) plan name
    plan_chosen = pyqtSignal(str)

    controller_reset_requested = pyqtSignal()
    start_requested = pyqtSignal()
    stop_requested = pyqtSignal()
    mirror_flip_requested = pyqtSignal()
    new_mirror_state = pyqtSignal(bool)
    odortest_requested = pyqtSignal(Odor)

    def __init__(self):
        super().__init__()

        # Main UI elements (from QtDesigner)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Fill hardware info combobox
        self.hardware_config_paths = []
        for dir in config.hardware_config_dirs:
            try:
                for path in files_in_dir(dir, 'yaml'):
                    self.hardware_config_paths.append(path)
                    self.ui.combo_hardware_config.addItem(
                        os.path.basename(path))
            except:
                sys.stderr.write(''.join(traceback.format_exc()))
                pass

        # Config displays
        self.inspect_hardware_model = QtGui.QStandardItemModel()
        self.ui.treeview_inspect_hardware.setModel(self.inspect_hardware_model)
        self.inspect_experiment_model = QtGui.QStandardItemModel()
        self.ui.treeview_inspect_experiment.setModel(self.inspect_experiment_model)

        # Step display setup
        self.step_model = QtCore.QStringListModel()
        self.ui.listview_plan.setModel(self.step_model)

        # Review pane
        self.reviews = []
        self.review_list_model = QtCore.QStringListModel()
        self.ui.listview_review_select.setModel(self.review_list_model)
        self.review_status_model = QtGui.QStandardItemModel()
        self.ui.treeview_exitstatus.setModel(self.review_status_model)

        # Signals/slots
        self.ui.button_experiment_config.clicked.connect(self.a_experiment)
        self.ui.button_configure.clicked.connect(self.a_configure)
        self.ui.combo_plan_select.currentIndexChanged.connect(self.a_set_plan)
        self.ui.button_plan_reload.clicked.connect(self.a_set_plan)
        self.ui.button_flipper.clicked.connect(self.a_flip_mirror)
        self.ui.checkbox_flipper.clicked.connect(self.a_set_mirror)
        self.ui.button_odortest.clicked.connect(self.a_odortest)
        self.ui.checkbox_odortest.clicked.connect(self.a_enable_odortest)
        self.ui.button_reset_controller.clicked.connect(self.a_reset_controller)
        self.ui.button_start.clicked.connect(self.a_startstop)
        self.ui.listview_review_select.selectionModel().currentChanged.connect(self.a_select_review)
        self.ui.textedit_extras.textChanged.connect(self.a_review_edited)
        self.ui.textedit_notes.textChanged.connect(self.a_review_edited)
        self.ui.combo_extras_format.currentIndexChanged.connect(self.a_set_extras_format)
        self.ui.button_load_review.clicked.connect(self.a_choose_load_review)
        self.ui.button_save_review.clicked.connect(self.a_save_current_review)

        # Various state trackers
        self.experiment_config_path = None
        self.configured = False
        self.run_in_progress = False
        self.reviews = []
        self.current_review = None

        self.show()
    
    def connect_worker(self, w: Worker):
        """Connect the worker's signals to our slots."""
        w.state_reset.connect(self.h_state_reset)
        w.reconfigured.connect(self.h_reconfigured)
        w.plan_generated.connect(self.h_plan_generated)
        w.error_encountered.connect(self.h_worker_error)
        w.panic_encountered.connect(self.h_worker_panic)
        w.run_manager.run_started.connect(self.h_run_started)
        w.run_manager.run_updated.connect(self.h_run_updated)
        w.run_manager.run_stopped.connect(self.h_run_stopped)
        w.run_manager.minirun_started.connect(self.h_minirun_started)
        w.run_manager.minirun_updated.connect(self.h_minirun_updated)
        w.run_manager.minirun_stopped.connect(self.h_minirun_stopped)
    

    # a_ methods are slots for ui elements

    def a_experiment(self, *args):
        """Choose an experiment config path with a file chooser."""
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self,
            'Choose an experiment config...',
            '', 'Config Files (*.yaml)'
        )
        if len(path) > 0:
            self.experiment_config_path = path
            self.ui.line_experiment_config.setText(path)
    
    def a_configure(self, *args):
        """Validate and send configuraton paths to the worker.

        Or, send a reset signal if a configuration is already loaded.
        """
        if not self.configured:
            index = self.ui.combo_hardware_config.currentIndex()
            hardware = self.hardware_config_paths[index]
            experiment = self.experiment_config_path
            if not experiment: return
            self.configs_chosen.emit(hardware, experiment)
        else:
            self.config_reset_requested.emit()
    
    def a_set_plan(self, *args):
        """Emit chosen plan name."""
        name = self.ui.combo_plan_select.currentText()
        self.plan_chosen.emit(name)
    
    def a_flip_mirror(self, *args):
        """Request the worker to flip the flipper mirror."""
        self.mirror_flip_requested.emit()

    def a_set_mirror(self, s):
        """Emit mirror state."""
        self.new_mirror_state.emit(s)
    
    def a_odortest(self, *args):
        """Request the worker to start an odor test."""
        self.odortest_requested.emit(Odor.from_string(
            self.ui.combo_odortest.currentText()))
    
    def a_enable_odortest(self, s):
        """Enable or disable the odortest button."""
        self.ui.button_odortest.setEnabled(s > 0)
        self.ui.combo_odortest.setEnabled(s > 0)
    
    def a_reset_controller(self, *args):
        """Request to reset the controller."""
        self.controller_reset_requested.emit()
    
    def a_startstop(self, *args):
        """Request to start or stop a run.

        (Depends on whether one is currently running.)
        """
        if self.run_in_progress:
            self.stop_requested.emit()
        else:
            self.start_requested.emit()
    
    def a_select_review(self, *args):
        """Fill the review pane with the selected review."""
        i = self.ui.listview_review_select.currentIndex().row()
        self.current_review = None
        self.show_review(self.reviews[i])
        self.current_review = self.reviews[i]
        self.ui.treeview_exitstatus.setEnabled(True)
        self.ui.textedit_extras.setEnabled(True)
        self.ui.textedit_notes.setEnabled(True)
    
    def a_review_edited(self, *args):
        """Sync the text in the review pane with our internal copy."""
        if self.current_review is None: return
        self.current_review.extras_raw = self.ui.textedit_extras.toPlainText()
        self.current_review.notes = self.ui.textedit_notes.toPlainText()
    
    def a_set_extras_format(self, *args):
        """Set the extras format for the current review."""
        if self.current_review is None: return
        self.current_review.extras_format = self.ui.combo_extras_format.currentText()
    
    def a_choose_load_review(self, *args):
        """Load a review from file.

        Includes both asking for a path (QFileDialog) and loading the file.
        """
        fd = QtWidgets.QFileDialog(self)
        fd.setFileMode(QtWidgets.QFileDialog.ExistingFile)
        fd.setNameFilter('Experiment Review (*.yaml *.json)')
        if not fd.exec(): return
        path = fd.selectedFiles()[0]

        try:
            ext = path.split('.')[-1].lower()
            with open(path) as fin:
                if ext == 'yaml':
                    import yaml
                    data = yaml.safe_load(fin)
                elif ext == 'json':
                    import json
                    data = json.load(fin)
                else:
                    self.show_error('Unrecognized review extension!')
                    return
            
            x = ReviewInfo(None)
            sd_keys = ['start-time', 'stop-time', 'aborted', 'exit-ok', 'plan']
            x.status_data = {}
            x.notes = ''
            extras = {}
            for k in data:
                if k in sd_keys:
                    x.status_data[k] = data[k]
                elif k != 'notes':
                    extras[k] = data[k]
                else:
                    x.notes = data[k]
            if ext == 'yaml':
                import yaml
                x.extras_raw = yaml.safe_dump(extras, None)
            else: #json
                import json
                x.extras_raw = json.dumps(extras)
            x.extras_format = ext.upper()

            self.add_review(x)
        except:
            self.show_error('Failed to load review!')
            self.show_exception(sys.exc_info())
    
    def a_save_current_review(self, *args):
        """Save the current review to file.

        Includes both asking for a path and saving to file.
        """
        if self.current_review is None: return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, 'Save...', './', 'Reviews (*.yaml *.json)')
        if not path: return

        try:
            r = self.current_review
            data = copy.copy(r.status_data)
            fmt = r.extras_format.lower()
            if fmt == 'yaml':
                import yaml
                exd = yaml.safe_load(r.extras_raw)
            else: #json
                import json
                exd = json.loads(r.extras_raw)
            if exd is not None:
                if not isinstance(exd, dict):
                    self.show_error("Extras must be a dict!")
                    return
                data |= exd
            data['notes'] = r.notes
            
            ext = path.split('.')[-1]
            if ext == 'json':
                import json
                with open(path, 'w') as fout: json.dump(data, fout, sort_keys=False)
            else:
                if ext != 'yaml': path += '.yaml'
                with open(path, 'w') as fout: yaml.safe_dump(data, fout, sort_keys=False)
        except:
            self.show_error('Failed to save review!')
            self.show_exception(sys.exc_info())


    # h_ methods are slots for Worker (and co.) signals
    
    def h_state_reset(self):
        """Slot for `Worker.state_reset`.

        Clear the UI to reflect that there is no loaded configuration;
        this also means that there is no plan, run, etc.
        """
        self.ui.button_configure.setText('Configure')
        self.ui.label_inspect_hardware.setText('(none)')
        self.ui.label_inspect_experiment.setText('(none)')
        self.ui.button_reset_controller.setEnabled(False)
        self.ui.button_start.setEnabled(False)
        self.ui.button_odortest.setEnabled(False)
        self.ui.checkbox_odortest.setEnabled(False)
        self.ui.button_flipper.setEnabled(False)
        self.ui.checkbox_flipper.setEnabled(False)
        self.ui.combo_plan_select.setEnabled(False)
        self.ui.button_plan_reload.setEnabled(False)
        self.inspect_hardware_model.clear()
        self.inspect_experiment_model.clear()
        self.step_model.setStringList([])
        self.configured = False
    
    def h_reconfigured(self, olf, pload):
        """Slot for `Worker.reconfigured`.

        Fill inspection models for the new olfactometer.
        """
        self.configured = True
        olf.mirror.state_changed.connect(self.h_mirror)

        # Fill plan selection combobox
        plans = list(pload.plans.keys())
        self.ui.combo_plan_select.clear()
        self.ui.combo_plan_select.addItems(plans)

        # Fill odortest combobox
        odors = olf.odors
        self.ui.combo_odortest.clear()
        self.ui.combo_odortest.addItems([str(o) for o in odors])

        # Show that the configuration has been loaded
        hardware, experiment = olf.source
        self.ui.button_configure.setText('Reset')
        self.ui.label_inspect_hardware.setText(os.path.basename(hardware))
        self.ui.label_inspect_experiment.setText(os.path.basename(experiment))

        # Enable actions that require configuration
        self.ui.button_reset_controller.setEnabled(True)
        self.ui.button_odortest.setEnabled(self.ui.checkbox_odortest.isChecked())
        self.ui.combo_odortest.setEnabled(self.ui.checkbox_odortest.isChecked())
        self.ui.checkbox_odortest.setEnabled(True)
        self.ui.button_flipper.setEnabled(True)
        self.ui.checkbox_flipper.setEnabled(True)
        self.ui.checkbox_flipper.setChecked(False)
        self.ui.combo_plan_select.setEnabled(True)
        self.ui.button_plan_reload.setEnabled(True)
        
        # TODO: fill in hardware model

        # Fill experiment model
        d = {}
        d['odors'] = olf.flow.pin2odor
        d['plans'] = pload.configs
        self.fill_itemmodel(self.inspect_experiment_model, d)
    
    def h_plan_generated(self, plan):
        """Slot for `Worker.plan_generated`.

        Preview the steps of the new plan.
        """
        self.show_plan(plan, None)
        self.ui.button_start.setEnabled(True)
    
    def h_worker_error(self, msg, exc):
        """Slot for `Worker.error_encountered`.
        
        Shows the message in a message box, and logs the exception.
        """
        if exc != (None, None, None):
            sys.stderr.write(''.join(traceback.format_exception(*exc)))
        self.show_error(msg)
    
    def h_worker_panic(self, exc):
        """Slot for `Worker.panic_encountered`.

        Log the exception, and exit the program.
        """
        sys.stderr.write(''.join(traceback.format_exception(*exc)))
        self.quit()
    
    def h_run_started(self, run):
        """Slot for `RunManager.run_started`.

        Set the timing info as 'reserved', so that minirun updates cannot
        influence it. Also update the UI to reflect that a run is in progress.
        """
        self.run_in_progress = True
        self.ui.button_start.setText('Stop')
        self.enable_inert_only_elements(False)
    
    def h_run_updated(self, state: RunState):
        """Slot for `RunManager.run_updated`.

        Update timing info and current step display.
        """
        self.show_plan(state.plan, state.step)
        self.show_timing(*state.timing)
    
    def h_run_stopped(self, status: RunExitStatus):
        """Slot for `RunManager.run_stopped`.

        Re-enable UI elements for configuring, planning, etc.
        """
        self.run_in_progress = False
        self.ui.button_start.setText('Start')
        self.enable_inert_only_elements(True)
        self.show_plan(status.plan, None)
        self.clear_timing()
        self.add_review(ReviewInfo(status))
        if not status.exit_ok:
            self.show_exc(status.exc_info)
            self.show_error('An error was encountered in the run!')
    
    def h_minirun_started(self, run):
        """Slot for `RunManager.minirun_started`."""
        if not self.run_in_progress:
            self.enable_inert_only_elements(False)
    
    def h_minirun_updated(self, state):
        """Slot for `RunManager.minirun_updated`."""
        if not self.run_in_progress:
            self.show_timing(*state.timing)
    
    def h_minirun_stopped(self, status):
        """Slot for `RunManager.minirun_stopped`."""
        if not self.run_in_progress:
            self.enable_inert_only_elements(True)
            self.clear_timing()
        if not status.exit_ok:
            self.show_exc(status.exc_info)
            self.show_error('An error was encountered in the minirun!')
    
    def h_mirror(self, x):
        """Slot for `MirrorManager.state_changed`."""
        if x != self.ui.checkbox_flipper.isChecked():
            self.ui.checkbox_flipper.setChecked(x)


    # Helper functions

    def show_plan(self, plan, hi):
        """Show the given `Plan`, highlighting step `hi` (or None)."""
        info = plan.info()
        if hi is not None:
            info[hi] = '>>> ' + info[hi]
        self.step_model.setStringList(info)
    
    def show_timing(self, step_prog, step_rem, total_prog, total_rem):
        """Fill in the timing bars and labels."""
        self.ui.pbar_step.setValue(int(round(100*step_prog)))
        self.ui.label_time_step.setText(self.format_dt(step_rem))
        self.ui.pbar_total.setValue(int(round(100*total_prog)))
        self.ui.label_time_total.setText(self.format_dt(total_rem))
    
    def clear_timing(self):
        """Reset the timing bars and labels."""
        self.ui.pbar_step.setValue(0)
        self.ui.label_time_step.setText('--:--')
        self.ui.pbar_total.setValue(0)
        self.ui.label_time_total.setText('--:--')

    def show_review(self, revinfo):
        """Fill the review pane."""
        self.fill_itemmodel(self.review_status_model, revinfo.status_data)
        self.ui.textedit_extras.setPlainText(revinfo.extras_raw)
        self.ui.textedit_notes.setPlainText(revinfo.notes)
        self.ui.combo_extras_format.setCurrentIndex({
            'YAML': 0,
            'JSON': 1,
        }[revinfo.extras_format])
    
    def show_error(self, msg):
        """Show an error message box."""
        box = QtWidgets.QMessageBox(self)
        box.setText(msg)
        box.setIcon(box.Warning)
        box.exec()
    
    def show_exc(self, exc):
        sys.stderr.write(''.join(traceback.format_exception(*exc)))
    
    def enable_inert_only_elements(self, x):
        """Enable or disable UI elements that can only be used when no run is in
        progress.
        
        If `x`, enable the UI elements; otherwise, disable them.
        """
        self.ui.button_configure.setEnabled(x)
        self.ui.combo_plan_select.setEnabled(x)
        self.ui.button_plan_reload.setEnabled(x)
    
    def add_review(self, revinfo):
        """Add a new run review."""
        self.reviews.append(revinfo)
        sl = self.review_list_model.stringList()
        sl.append(f'Run from {revinfo.status_data["stop-time"]}')
        self.review_list_model.setStringList(sl)
        self.ui.listview_review_select.selectionModel().select(
            self.review_list_model.index(len(sl)-1),
            self.ui.listview_review_select.selectionModel().Select
        )

    def quit(self):
        """Shut down the worker, and quit the application."""
        g_worker_thread.quit()
        g_worker_thread.wait()
        g_app.quit()

    def closeEvent(self, event):
        """Handler for the X button.

        Ask for confirmation, then `quit()` if it is given.
        """
        MB = QtWidgets.QMessageBox
        res = MB.question(
            self,
            'Quit?', 'Are you sure you want to quit?',
            MB.Cancel | MB.Ok, MB.Ok
        )
        if res == MB.Ok:
            self.quit()
        else:
            event.ignore()

    @staticmethod
    def fill_itemmodel(model: QtGui.QStandardItemModel, data):
        """Fill a Qt5 `QStandardItemModel` with nested data.

        The `data` object is a dictionary containing POD (numbers, strings)
        and nested structures (lists, dicts). The `model` is cleared before
        any new data is added.
        """
        model.clear()
        for br in UI.qtree_from_dicts(data):
            model.appendRow(br)
    
    @staticmethod
    def qtree_from_dicts(data):
        """Helper function for `UI.fill_itemmodel()`.

        Recursively constructs `QStandardItem` branches from `data`.
        Return a list of branches.
        """
        QSI = QtGui.QStandardItem
        branches = []
        for k in data.keys():
            v = data[k]
            asdict = None
            if isinstance(v, dict):
                asdict = v
            elif isinstance(v, list):
                asdict = {f'#{i+1}' : x for i, x in enumerate(v)}
            if asdict is not None:
                sub = UI.qtree_from_dicts(asdict)
                row = QSI(str(k))
                for s in sub:
                    row.appendRow(s)
                row = [row, QSI('')]
            else:
                row = [QSI(str(k)), QSI(str(v))]
            branches.append(row)
        return branches
    
    @staticmethod
    def format_dt(dt):
        """Format dt in seconds as mm:ss. Helper for UI.show_plan_state."""
        m = int(dt/60)
        s = int(round(dt%60))
        return f'{m:02}:{s:02}'
    

def main():
    global g_app
    global g_worker_thread

    g_app = QtWidgets.QApplication(sys.argv)
    ui = UI()

    g_worker_thread = QtCore.QThread()
    worker = Worker()
    worker.moveToThread(g_worker_thread)

    worker.connect_ui(ui)
    ui.connect_worker(worker)

    g_worker_thread.started.connect(worker.begin)
    g_worker_thread.finished.connect(worker.quit)
    g_worker_thread.start()

    g_app.exec()
    return 0

if __name__ == '__main__':
    sys.exit(main())