"""Scene consumer lifecycle: applied version, operation capability, and expiry.

Document application and Reset/planner operation are separate facts. applied and
operational are explicit; success is never an input authority. A live consumer that
understood this epoch/revision is synchronized even if it cannot operate.
unsupported is a declared capability gap, not a retryable transport failure.
Exited members expire by receipt time, not by deleting a still-reporting failure.
"""

import copy
import time


CAPABILITY_OK = 'ok'
CAPABILITY_UNSUPPORTED = 'unsupported'
LIVE_TIMEOUT = 3.0
CAPABILITIES = ('', CAPABILITY_OK, CAPABILITY_UNSUPPORTED)


def _finite_stamp(value):
    try:
        stamp = float(value)
    except (TypeError, ValueError):
        return 0.0
    if stamp != stamp or stamp < 0.0:  # NaN
        return 0.0
    return stamp


def _required_bool(report, name):
    if name not in report:
        raise ValueError('consumer {} is required'.format(name))
    value = report[name]
    if not isinstance(value, bool):
        raise ValueError('consumer {} must be a boolean'.format(name))
    return value


def normalize(report):
    if not isinstance(report, dict):
        raise ValueError('consumer report must be an object')
    name = report.get('consumer')
    if not isinstance(name, str) or not name:
        raise ValueError('consumer identity is required')
    epoch = report.get('epoch')
    if epoch is None:
        epoch = ''
    if not isinstance(epoch, str):
        raise ValueError('consumer epoch must be a string')
    revision = report.get('revision', 0)
    if isinstance(revision, bool) or not isinstance(revision, (int, float)):
        raise ValueError('consumer revision must be an integer')
    revision = int(revision)
    if revision < 0:
        raise ValueError('consumer revision must be nonnegative')
    applied = _required_bool(report, 'applied')
    operational = _required_bool(report, 'operational')
    capability = report.get('capability')
    if capability is None:
        capability = ''
    if not isinstance(capability, str):
        raise ValueError('consumer capability must be a string')
    if capability not in CAPABILITIES:
        raise ValueError('unknown consumer capability {}'.format(capability))
    generation = report.get('generation', 0) or 0
    if isinstance(generation, bool) or not isinstance(generation, (int, float)):
        raise ValueError('consumer generation must be an integer')
    generation = int(generation)
    if generation < 0:
        raise ValueError('consumer generation must be nonnegative')
    message = report.get('message') or ''
    if not isinstance(message, str):
        raise ValueError('consumer message must be a string')
    return {
        'consumer': name,
        'epoch': epoch,
        'revision': revision,
        'applied': applied,
        'operational': operational,
        'capability': capability,
        'generation': generation,
        'message': message,
        'header_stamp': _finite_stamp(report.get('header_stamp')),
    }


def synchronized(consumers, epoch, revision):
    live = list(consumers)
    if not live:
        return True
    return all(item['applied'] and item['epoch'] == epoch and item['revision'] == revision
               for item in live)


def can_retry_sync(consumers, epoch, revision):
    """Retry only helps version lag or failed application, not a declared capability gap."""
    if synchronized(consumers, epoch, revision):
        return False
    lagged = [item for item in consumers if item['epoch'] != epoch or item['revision'] != revision]
    if lagged:
        return True
    missing = missing_apply(consumers, epoch, revision)
    if not missing:
        return False
    return any(item['capability'] != CAPABILITY_UNSUPPORTED for item in missing)


def missing_apply(consumers, epoch, revision):
    return [item for item in consumers
            if item['epoch'] == epoch and item['revision'] == revision and not item['applied']]


class ConsumerRegistry:
    def __init__(self, timeout=LIVE_TIMEOUT, clock=time.monotonic):
        if not isinstance(timeout, (int, float)) or isinstance(timeout, bool) or timeout <= 0:
            raise ValueError('consumer timeout must be positive')
        self.timeout = float(timeout)
        self.clock = clock
        self._items = {}

    def update(self, report, now=None):
        record = normalize(report)
        received = self.clock() if now is None else float(now)
        previous = self._items.get(record['consumer'])
        if previous is not None:
            if record['generation'] and previous['generation'] and record['generation'] < previous['generation']:
                return previous
            if record['header_stamp'] and previous['header_stamp'] and record['header_stamp'] < previous['header_stamp']:
                return previous
        record['received_at'] = received
        self._items[record['consumer']] = record
        return record

    def expire(self, now=None):
        received = self.clock() if now is None else float(now)
        expired = [name for name, item in self._items.items()
                   if received - item['received_at'] > self.timeout]
        for name in expired:
            del self._items[name]
        return expired

    def live(self, now=None):
        self.expire(now)
        return [copy.deepcopy(item) for item in self._items.values()]

    def public(self, epoch, revision, now=None):
        consumers = []
        for item in self.live(now):
            consumers.append({
                'consumer': item['consumer'],
                'epoch': item['epoch'],
                'revision': item['revision'],
                'applied': item['applied'],
                'operational': item['operational'] and item['capability'] != CAPABILITY_UNSUPPORTED,
                'capability': item['capability'],
                'generation': item['generation'],
                'message': item['message'],
                # Derived publish of applied. Not a second authority.
                'success': item['applied'],
            })
        consumers.sort(key=lambda item: item['consumer'])
        return {
            'consumers': consumers,
            'synchronized': synchronized(consumers, epoch, revision),
            'syncRetryable': can_retry_sync(consumers, epoch, revision),
        }
