import os
import sys
import unittest

import ament_index_python

import launch
import launch.actions

import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertSequentialStdout

import pytest

@pytest.mark.launch_test
def generate_test_description():
    TEST_PROC_PATH = os.path.join(
        ament_index_python.get_package_prefix('HelloLaunch'),
        'lib', 'HelloLaunch',
        'hello_launch'
    )

    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    dut_process = launch.actions.ExecuteProcess(
        cmd=[TEST_PROC_PATH],
        env=proc_env, output='screen'
    )

    return launch.LaunchDescription([
        dut_process,

        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest(),
    ]), {'dut_process': dut_process}


class TestClass(unittest.TestCase):

    def test_count_to_four(self, proc_output):
        # This will match stdout from any process.  In this example there is only one process
        # running
        proc_output.assertWaitFor("Publishing: 'Hello, world! 0'", timeout=10, stream='stdout')

@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # Check that all processes in the launch (in this case, there's just one) exit
        # with code 0
        launch_testing.asserts.assertExitCodes(proc_info)

    def test_full_output(self, proc_output, dut_process):
        # Using the SequentialStdout context manager asserts that the following stdout
        # happened in the same order that it's checked
        with assertSequentialStdout(proc_output, dut_process) as cm:
            cm.assertInStdout('Starting Up')
            for n in range(4):
                cm.assertInStdout('Loop {}'.format(n))
            if os.name != 'nt':
                # On Windows, process termination is always forced
                # and thus the last print in good_proc never makes it.
                cm.assertInStdout('Shutting Down')

    def test_out_of_order(self, proc_output, dut_process):
        # This demonstrates that we notice out-of-order IO
        with self.assertRaisesRegex(AssertionError, "'Loop 2' not found"):
            with assertSequentialStdout(proc_output, dut_process) as cm:
                cm.assertInStdout('Loop 1')
                cm.assertInStdout('Loop 3')
                cm.assertInStdout('Loop 2')  # This should raise
