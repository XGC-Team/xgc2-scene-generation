import time
import unittest

from xgc2_scene_runtime.consumers import (
    CAPABILITY_UNSUPPORTED,
    ConsumerRegistry,
    can_retry_sync,
    normalize,
    synchronized,
)


class ConsumerRegistryTest(unittest.TestCase):
    def test_success_is_not_an_applied_authority(self):
        with self.assertRaisesRegex(ValueError, 'applied is required'):
            normalize({'consumer': 'gazebo', 'epoch': 'e', 'revision': 2, 'success': True,
                       'message': 'applied'})
        record = normalize({
            'consumer': 'gazebo', 'epoch': 'e', 'revision': 2, 'success': True,
            'applied': False, 'operational': False, 'message': 'stale field',
        })
        self.assertFalse(record['applied'])
        self.assertFalse(record['operational'])
        self.assertNotIn('success', record)

    def test_capability_gap_does_not_count_as_retryable_unsync(self):
        consumers = [
            {'consumer': 'gazebo', 'epoch': 'e', 'revision': 3, 'applied': True, 'operational': True,
             'capability': '', 'success': True},
            {'consumer': 'ugv-reset', 'epoch': 'e', 'revision': 3, 'applied': False,
             'operational': False, 'capability': CAPABILITY_UNSUPPORTED, 'success': False},
        ]
        self.assertFalse(synchronized(consumers, 'e', 3))
        self.assertFalse(can_retry_sync(consumers, 'e', 3))

    def test_version_lag_and_apply_failure_remain_retryable(self):
        lag = [{'consumer': 'gazebo', 'epoch': 'e', 'revision': 2, 'applied': True,
                'operational': True, 'capability': '', 'success': True}]
        self.assertFalse(synchronized(lag, 'e', 3))
        self.assertTrue(can_retry_sync(lag, 'e', 3))
        failed = [{'consumer': 'gazebo', 'epoch': 'e', 'revision': 3, 'applied': False,
                   'operational': False, 'capability': '', 'success': False}]
        self.assertTrue(can_retry_sync(failed, 'e', 3))

    def test_applied_capability_limit_still_counts_as_document_sync(self):
        consumers = [
            {'consumer': 'gazebo', 'epoch': 'e', 'revision': 1, 'applied': True, 'operational': True,
             'capability': '', 'success': True},
            {'consumer': 'ugv-reset', 'epoch': 'e', 'revision': 1, 'applied': True,
             'operational': False, 'capability': CAPABILITY_UNSUPPORTED, 'success': True},
        ]
        self.assertTrue(synchronized(consumers, 'e', 1))
        self.assertFalse(can_retry_sync(consumers, 'e', 1))

    def test_exited_consumers_expire_and_reentry_replaces_them(self):
        clock = [10.0]
        registry = ConsumerRegistry(timeout=1.0, clock=lambda: clock[0])
        registry.update({'consumer': 'ugv-reset', 'epoch': 'old', 'revision': 1,
                          'applied': False, 'operational': False, 'message': 'gone',
                          'generation': 1, 'header_stamp': 1.0})
        clock[0] = 12.0
        self.assertEqual(registry.live(), [])
        registry.update({'consumer': 'ugv-reset', 'epoch': 'new', 'revision': 4,
                          'applied': True, 'operational': True, 'capability': 'ok',
                          'generation': 2, 'header_stamp': 5.0})
        live = registry.live()
        self.assertEqual(len(live), 1)
        self.assertEqual(live[0]['epoch'], 'new')
        self.assertEqual(live[0]['generation'], 2)

    def test_old_epoch_and_out_of_order_stamps_do_not_overwrite(self):
        clock = [1.0]
        registry = ConsumerRegistry(timeout=10.0, clock=lambda: clock[0])
        registry.update({'consumer': 'ugv-reset', 'epoch': 'e2', 'revision': 4,
                          'applied': True, 'operational': True, 'generation': 3,
                          'header_stamp': 8.0})
        clock[0] = 1.1
        registry.update({'consumer': 'ugv-reset', 'epoch': 'e1', 'revision': 1,
                          'applied': False, 'operational': False, 'generation': 2,
                          'header_stamp': 9.0})
        self.assertEqual(registry.live()[0]['epoch'], 'e2')
        registry.update({'consumer': 'ugv-reset', 'epoch': 'e2', 'revision': 3,
                          'applied': False, 'operational': False, 'generation': 3,
                          'header_stamp': 7.0})
        self.assertEqual(registry.live()[0]['revision'], 4)

    def test_public_envelope_splits_sync_from_capability_and_derives_success(self):
        registry = ConsumerRegistry(timeout=5.0, clock=lambda: 1.0)
        registry.update({'consumer': 'gazebo', 'epoch': 'e', 'revision': 2,
                          'applied': True, 'operational': True, 'capability': 'ok',
                          'message': 'applied', 'header_stamp': 1.0})
        registry.update({'consumer': 'ugv-reset', 'epoch': 'e', 'revision': 2,
                          'applied': True, 'operational': False, 'capability': 'unsupported',
                          'message': 'unknown motion', 'header_stamp': 1.0})
        public = registry.public('e', 2)
        self.assertTrue(public['synchronized'])
        self.assertFalse(public['syncRetryable'])
        reset = next(item for item in public['consumers'] if item['consumer'] == 'ugv-reset')
        self.assertTrue(reset['applied'])
        self.assertFalse(reset['operational'])
        self.assertEqual(reset['capability'], 'unsupported')
        self.assertEqual(reset['success'], reset['applied'])
        gazebo = next(item for item in public['consumers'] if item['consumer'] == 'gazebo')
        self.assertTrue(gazebo['success'])
        self.assertTrue(gazebo['applied'])


if __name__ == '__main__':
    unittest.main()
