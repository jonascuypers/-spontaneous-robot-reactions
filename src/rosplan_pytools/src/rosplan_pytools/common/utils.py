from diagnostic_msgs.msg import KeyValue
from rosplan_knowledge_msgs.msg import KnowledgeItem
import rospy
from std_srvs.srv import Empty
from rosplan_dispatch_msgs.srv import DispatchService

def keyval_to_dict(keyval_list):
    if keyval_list is None or keyval_list == []:
        return {}
    out = {}
    for item in keyval_list:
        out[item.key] = item.value
    return out


def dict_to_keyval(in_dict):
    if in_dict is None:
        return []
    out = []
    for item in in_dict.iteritems():
        out.append(KeyValue(*item))
    return out


def predicate_maker(attr_name, attr_key, attr_value, is_negative=None):
    ki = KnowledgeItem()
    ki.knowledge_type = 1
    ki.attribute_name = attr_name
    ki_values = []
    if type(attr_key) == list:
        for key,value in zip(attr_key,attr_value):
            kv = KeyValue()
            kv.key = key
            kv.value = value
            ki_values.append(kv)
    else:
        kv = KeyValue()
        kv.key = attr_key
        kv.value = attr_value
        ki_values.append(kv)
    ki.values = ki_values
    if is_negative:
        ki.is_negative = is_negative
    return ki


def plan_and_execute():
    rospy.wait_for_service('rosplan_problem_interface/problem_generation_server')
    rospy.wait_for_service('rosplan_planner_interface/planning_server')
    rospy.wait_for_service('rosplan_parsing_interface/parse_plan')
    rospy.wait_for_service('rosplan_plan_dispatcher/dispatch_plan')
    rospy.ServiceProxy('rosplan_problem_interface/problem_generation_server', Empty)()
    rospy.ServiceProxy('rosplan_planner_interface/planning_server', Empty)()
    rospy.ServiceProxy('rosplan_parsing_interface/parse_plan', Empty)()
    rospy.ServiceProxy('rosplan_plan_dispatcher/dispatch_plan', DispatchService)()
