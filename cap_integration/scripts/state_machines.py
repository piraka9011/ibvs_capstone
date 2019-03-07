#!/usr/bin/env python
import sys
sys.path.insert(0, '/home/piraka/sawyer_ws/src/ibvs_capstone/perception')
from rospy_message_converter import message_converter
import rospy
import smach
import smach_ros
import threading
from std_msgs.msg import String
from dialogflow_ros.msg import DialogflowResult, DialogflowParameter
from perception.msg import YoloObject, YoloObjectList
import signal

def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        sys.exit(0)


class WaitForMsgState(smach.State):
    """
    Waits for a message and routes the contents to next State.
    """

    def __init__(self, topic, msg_type, msg_cb=None, output_keys=[], input_keys=[], latch=False, timeout=None,
                 state_name=None):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], output_keys=output_keys, input_keys=input_keys)
        self.latch = latch
        self.timeout = timeout
        self.mutex = threading.Lock()
        self.msg = None
        self.message_callback = msg_cb
        self.subscriber = rospy.Subscriber(topic, msg_type, self._callback, queue_size=1)
        self.state_name = state_name

    def _callback(self, msg):
        self.mutex.acquire()
        self.msg = msg
        self.mutex.release()

    def wait_for_msg(self):
        rospy.loginfo(self.state_name + ' is waiting for message...')
        if self.timeout is not None:
            timeout_time = rospy.Time.now() + rospy.Duration.from_sec(self.timeout)
        while self.timeout is None or rospy.Time.now() < timeout_time:
            self.mutex.acquire()
            if self.msg is not None:
                rospy.loginfo(self.state_name + ' got message.')
                message = self.msg

                if not self.latch:
                    self.msg = None

                self.mutex.release()
                return message
            self.mutex.release()

            #TODO: Do we need a preempted out?

            # if self.preempt_requested():
            #     self.service_preempt()
            #     rospy.loginfo('waitForMsg is preempted!')
            #     return 'preempted'

            rospy.sleep(.1) # TODO: InterruptException? Discuss with Anas

        rospy.loginfo(self.state_name + ' experienced a Timeout on waiting for message.')
        return None

    def execute(self, ud):
        msg = self.wait_for_msg()
        if msg is not None:
            if msg == 'preempted':
                return 'preempted'
            if self.message_callback is not None:
                ud.publish_msg = self.msg
                cb_result = self.message_callback(msg, ud)
                if cb_result is not None:
                    if cb_result:
                        return 'succeeded'
                    else:
                        return 'aborted'
            return 'succeeded'
        else:
            return 'aborted'


class PublishMsgState(smach.State):
    def __init__(self, publisher_name, node_name=None, message_type=String, output_keys=[], input_keys=[],
                 state_name=None):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], output_keys=output_keys, input_keys=input_keys)
        self.pub = rospy.Publisher(publisher_name, message_type, queue_size=1)
        if node_name is not None:
            rospy.init_node(node_name, anonymous=True)
        self.state_name = state_name

    def execute(self, ud):
        rospy.loginfo("Publishing " + self.state_name)
        self.pub.publish(ud.publish_msg)
        return 'succeeded'

def amber_result(msg, ud):
  objects = msg.objects#ud.obj_list
  for obj in objects:
      if obj.tag.lower() == ud.obj.value[0].lower():
          #rospy.loginfo("In if statement")
          ud.publish_msg = obj
      #rospy.loginfo(obj.tag.lower())
      #rospy.loginfo(ud.obj.value[0].lower())
  rospy.loginfo(objects)

def dialogflowresult_format(msg, ud):
    ud.action = msg.action
    ud.parameters = msg.parameters
    ud.obj = msg.parameters[0]
    ud.publish_msg = "start"
    rospy.loginfo(ud.action)
    return True


def main():
    rospy.init_node('integration')

    yolo_target_topic = rospy.get_param('/camera/color/image_rect_color', '/ibvs/perception/yolo_target')
    
    sm_top = smach.StateMachine(outcomes=['success'])
    sm_top.userdata.parameters = []
    sm_top.userdata.action = ""
    sm_top.userdata.obj_list = YoloObjectList()
    sm_top.userdata.obj = ""
    sm_top.userdata.publish_msg = {}

    with sm_top:

        smach.StateMachine.add('NLPsub', WaitForMsgState('/dialogflow_client/results', DialogflowResult,
                                                         msg_cb=dialogflowresult_format, state_name="NLPsub",
                                                         output_keys=['publish_msg', 'action', 'parameters', 'obj_list',
                                                            'obj'],
                                                         input_keys=['publish_msg', 'action', 'parameters', 'obj_list',
                                                            'obj']),
                               transitions={'succeeded': 'NLPpub',
                                            'aborted': 'NLPsub'})

        smach.StateMachine.add('NLPpub', PublishMsgState('/nlp', state_name='NLPpub',
                                                         output_keys=['publish_msg', 'action', 'parameters'],
                                                         input_keys=['publish_msg', 'action', 'parameters']),
                               transitions={'succeeded': 'AMBERsub',
                                            'aborted': 'NLPpub'})
        smach.StateMachine.add('AMBERsub', WaitForMsgState('/amber', YoloObjectList, state_name="AMBERsub",
                                                           output_keys=['publish_msg', 'action', 'parameters',
                                                            'obj_list', 'obj'],
                                                           input_keys=['publish_msg', 'action', 'parameters', 'obj_list',
                                                            'obj'], msg_cb=amber_result),
                               transitions={'succeeded': 'JAKEpub',
                                            'aborted': 'AMBERsub'})
        smach.StateMachine.add('AMBERpub', PublishMsgState('/amber', state_name='AMBERpub',
                                                           output_keys=['publish_msg', 'action', 'parameters'],
                                                           input_keys=['publish_msg', 'action', 'parameters']),
                               transitions={'succeeded': 'JAKEsub',
                                            'aborted': 'AMBERsub'})
        smach.StateMachine.add('JAKEsub', WaitForMsgState('/amber', YoloObject, state_name='JAKEsub',
                                                          output_keys=['publish_msg', 'action', 'parameters'],
                                                          input_keys=['publish_msg', 'action', 'parameters']),
                               transitions={'succeeded': 'JAKEpub',
                                            'aborted': 'JAKEsub'})
        smach.StateMachine.add('JAKEpub', PublishMsgState('/jake', state_name='JAKEpub', message_type=YoloObject,
                                                          output_keys=['publish_msg', 'action', 'parameters'],
                                                          input_keys=['publish_msg', 'action', 'parameters']),
                               transitions={'succeeded': 'ANASpub',
                                            'aborted': 'JAKEpub'})
        smach.StateMachine.add('ANASsub', WaitForMsgState('/jake', YoloObject, state_name='ANASsub',
                                                          output_keys=['publish_msg', 'action', 'parameters'],
                                                          input_keys=['publish_msg', 'action', 'parameters']),
                               transitions={'succeeded': 'ANASpub',
                                            'aborted': 'ANASsub'})
        # TODO: Change the message_type from the one from jake by changing message_type
        smach.StateMachine.add('ANASpub', PublishMsgState('/anas', state_name='ANASpub', message_type=YoloObject,
                                                          output_keys=['publish_msg', 'action', 'parameters'],
                                                          input_keys=['publish_msg', 'action', 'parameters']),
                               transitions={'succeeded': 'success',
                                            'aborted': 'ANASpub'})




    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm_top.execute()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    main()
