from enum import Enum

from tello_msgs.msg import TelloResponse
from tello_msgs.srv import TelloAction


class ActionMgr(object):
    """
    tello_driver uses a ROS service plus a ROS topic to simulate an action.
    This should be replaced when the rclpy action API appears in ROS2 Dashing.

    Overall flow:
        1. flock_base sends a TelloAction.Request on the tello_action service
        2. the drone responds with a TelloAction.Response
        3. later, the drone sends a TelloResponse message on the tello_response topic

    Only one action can be active at a time.

    TODO time out if tello_driver hasn't responded in ~10s, this can happen if tello_driver was restarted
    """

    class States(Enum):
        NOT_SENT = 0                    # TelloAction.Request not sent yet
        WAITING_FOR_FUTURE = 1          # Waiting for TelloAction.Response
        WAITING_FOR_RESPONSE = 2        # Waiting for TelloResponse
        SUCCEEDED = 3                   # Action succeeded, see the result string
        FAILED = 4                      # Action failed, see the result string
        FAILED_LOST_CONNECTION = 5      # Action failed, there's no connection to the drone

    def __init__(self, logger, action_client, action_code, action_str):
        self.logger = logger                        # Node.get_logger()
        self.action_client = action_client          # Node.create_client(...)
        self.action_code = action_code              # Action code, for reference
        self.action_str = action_str                # Action string, will be sent to the drone
        self.state = ActionMgr.States.NOT_SENT      # Internal state
        self.future = None                          # Future returned from client.call_async
        self.result_str = None                      # Result string

    def advance(self) -> States:
        """
        Call periodically to advance the state
        """
        if self.state == ActionMgr.States.NOT_SENT:
            self.logger.debug('send {} to tello_driver'.format(self.action_str))
            self.future = self.action_client.call_async(TelloAction.Request(cmd=self.action_str))
            self.state = ActionMgr.States.WAITING_FOR_FUTURE

        if self.state == ActionMgr.States.WAITING_FOR_FUTURE and self.future.done():
            response: TelloAction.Response = self.future.result()
            self.future = None

            if response.rc == TelloAction.Response.OK:
                self.logger.debug('{} accepted'.format(self.action_str))
                self.state = ActionMgr.States.WAITING_FOR_RESPONSE

            elif response.rc == TelloAction.Response.ERROR_BUSY:
                self.logger.error('{} failed, drone is busy'.format(self.action_str))
                self.result_str = 'drone is busy'
                self.state = ActionMgr.States.FAILED

            elif response.rc == TelloAction.Response.ERROR_NOT_CONNECTED:
                self.logger.error('{} failed, lost connection'.format(self.action_str))
                self.result_str = 'lost connection'
                self.state = ActionMgr.States.FAILED_LOST_CONNECTION

        return self.state

    def complete(self, msg: TelloResponse) -> States:
        """
        The tello_response message may arrive before self._future.done() evaluates True -- that's OK
        """
        if self.state != ActionMgr.States.WAITING_FOR_FUTURE and self.state != ActionMgr.States.WAITING_FOR_RESPONSE:
            self.logger.error('unexpected response {}'.format(msg))
            self.result_str = 'unexpected response'
            self.state = ActionMgr.States.FAILED

        elif msg.rc == TelloResponse.OK:
            self.logger.debug('{} succeeded with {}'.format(self.action_str, msg.str))
            self.result_str = msg.str
            self.state = ActionMgr.States.SUCCEEDED

        elif msg.rc == TelloResponse.ERROR:
            self.logger.error('{} failed with {}'.format(self.action_str, msg.str))
            self.result_str = msg.str
            self.state = ActionMgr.States.FAILED

        elif msg.rc == TelloResponse.TIMEOUT:
            self.logger.error('{} failed, drone timed out'.format(self.action_str))
            self.result_str = 'tello_driver timed out'
            self.state = ActionMgr.States.FAILED

        return self.state
