from errors import *
import random
import copy

def order_from_spec(spec):
    if isinstance(spec, str):
        return [spec]
    elif isinstance(spec, list):
        shuffle = False
        items = spec
    else:
        assert isinstance(spec, dict)
        shuffle = spec.get('shuffle', False)
        items = spec['items']
    items = copy.copy(items) # shallow ok
    if shuffle:
        random.shuffle(items)
    out = []
    for x in items:
        out += order_from_spec(x)
    return out

def default_order_spec(names):
    pfo = None
    if 'pfo' in names: pfo = 'pfo'
    elif 'paraffin' in names: pfo = 'paraffin'
    if pfo is None: return [names]
    return [pfo, {'shuffle': True, 'items': [x for x in names if x != pfo]}]

def try_typecasting(cfg, *args):
    a = []
    for k, ty in args:
        try:
            a.append(ty(cfg[k]))
        except:
            raise ParseError(f'Failed to typecast param: {k}')
    return tuple(a)

def deep_update(a, b):
    for k, v in b.items():
        if k not in a:
            a[k] = v
        else:
            va = a[k]
            if isinstance(va, dict):
                assert isinstance(v, dict)
                deep_update(va, v)
            else:
                a[k] = v

class VialOdor:
    def __init__(self, odors, carrier_rate, odor_rate):
        self.order = sorted(odors, key=lambda o: o.conc_num())
        self.carrier_rate = carrier_rate
        self.odor_rate = odor_rate
    
    def insert_steps(self, plan, prestimt, poststimt, stimt):
        for o in self.order:
            plan.push(f'Odor: {o}')
            plan += [
                AdjustOdorFlowStep(o, self.carrier_rate, self.odor_rate),
                WaitStep(prestimt),
                OdorStep(o, stimt),
                WaitStep(poststimt)
            ]
            plan.pop()

def parse_float_or_percent(x, of_what):
    if isinstance(x, str):
        assert x[-1] == '%'
        prop = float(x[:-1])/100
        return prop*of_what
    else:
        return float(x)

class GasOdor:
    def __init__(self, odor, cfg):
        self.odor = odor
        tot = float(cfg['total-flow'])
        oflows = []
        for x in cfg['odor-flows']:
            oflows.append(parse_float_or_percent(x, tot))
        cflows = [tot-x for x in oflows]
        self.rates = sorted(
            list(zip(cflows, oflows)),
            key = lambda x: x[1]
        )
    
    def insert_steps(self, plan, prestimt, poststimt, stimt):
        for carrier_rate, odor_rate in self.rates:
            pc = round(100*odor_rate/(odor_rate+carrier_rate))
            plan.push(f'Odor: {self.odor} at {pc}%')
            plan += [
                AdjustOdorFlowStep(self.odor, carrier_rate, odor_rate),
                WaitStep(prestimt),
                OdorStep(self.odor, stimt),
                WaitStep(poststimt)
            ]

@plan('Concentration Ramp', 'ramp')
def concentration_ramp_plan(odors, cfg_):
    names = list(set([o.name for o in odors]))
    cfg = {
        'order': default_order_spec(names),
        'gasses': {},
        'nblocks': 3,
        'timing': {
            'stim': 2.0,
            'begin-movie': 30.0,
            'pre-stim': 30.0,
            'post-stim': 30.0,
            'rest': 120.0
        },
        'default-flow': {
            'total-flow': 2000.0,
            'odor-flow': '10%'
        }
    }
    deep_update(cfg, cfg_)

    nb, timing = try_typecasting(cfg,
        ('nblocks', int), ('timing', dict)
    )

    stimt, begint, prestimt, poststimt, restt = try_typecasting(timing,
        ('stim', float), ('begin-movie', float),
        ('pre-stim', float), ('post-stim', float),
        ('rest', float)
    )

    default_flow_cfg = dict(cfg['default-flow'])
    vial_total_rate, = try_typecasting(default_flow_cfg, ('total-flow', float))
    vial_odor_rate = parse_float_or_percent(default_flow_cfg['odor-flow'], vial_total_rate)
    vial_carrier_rate = vial_total_rate - vial_odor_rate

    order_spec = order_from_spec(cfg['order'])
    order = []
    for x in order_spec:
        assert x in names
        relevant = [o for o in odors if o.name == x]
        if x in cfg['gasses']:
            assert len(relevant) == 1
            order.append(GasOdor(relevant[0], cfg['gasses'][x]))
        else:
            assert len(relevant) > 0
            order.append(VialOdor(relevant, vial_carrier_rate, vial_odor_rate))
    
    sl = StepList()
    for i in range(nb):
        sl.push(f'Block {i+1}')
        sl += [
            MirrorDownStep(),
            ScopeOnStep()
        ]
        if i == 0:
            sl.append(WaitStep(begint))
        for o in order:
            o.insert_steps(sl, prestimt, poststimt, stimt)
        sl += [
            ScopeOffStep(),
            MirrorUpStep()
        ]
        if i+1 < nb:
            sl.append(WaitStep(restt))
        sl.pop()
    
    return sl