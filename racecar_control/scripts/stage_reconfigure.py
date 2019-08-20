#!/usr/bin/env python
import os
import rospy
import rospkg
import yaml
from racecar_msgs.msg import Stage
import dynamic_reconfigure.client
from std_srvs.srv import Empty

class StageReconfigure:
  def __init__(self):
    self.config = {}
    self.nodes = {}
    self.clients = {}
    self.last_stage = Stage()
    self.last_stage.stage = -1
    self.stage_sub = rospy.Subscriber('/stage', Stage, self.stageCallback)

  def load(self):
    rospack = rospkg.RosPack()
    rospack.list()
    pkg_path = rospack.get_path('racecar_control')

    config_file = os.path.join(pkg_path, 'config', 'stage_reconfigure.yaml')
    config = {}
    with open(config_file, 'r') as f:
      try:
        config = yaml.safe_load(f)
      except yaml.YAMLError as ex:
        raise ex

    self.config = config.pop('config')

    for node in config:
      stages = config[node].pop('stages')
      params = {}

      for stage in stages:
        stage_key = stage.pop('stage')
        enter_params = stage.pop('_enter', {})
        enter_distance = enter_params.pop('_distance', 0.0)
        exit_params = stage.pop('_exit', {})
        exit_distance = exit_params.pop('_distance', 0.0)

        clear_costmap = stage.pop('clear_costmap', False)

        # merge params with upper params
        enter = stage.copy()
        enter.update(enter_params)
        exit = stage.copy()
        exit.update(exit_params)

        stage_info = {
          'params': stage,
          'enter': enter,
          'enter_distance': enter_distance,
          'exit': exit,
          'exit_distance': exit_distance,
          'clear_costmap': clear_costmap,
        }

        if not isinstance(stage_key, list):
          params[stage_key] = stage_info
        else:
          for key in stage_key:
            params[key] = stage_info

      self.nodes[node] = {
        'config': config[node],
        'stages': params,
      }

      rospy.loginfo('node %s load %s params' % (node, len(params)))

  def update(self, node, params):
    if node not in self.clients:
      self.clients[node] = dynamic_reconfigure.client.Client(node, timeout=3)
    self.clients[node].update_configuration(params)

  def clear_costmaps(self):
    try:
      rospy.wait_for_service('/move_base/clear_costmaps', 1)
      clear_func = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
      clear_func()
      print 'costmaps cleared'
    except rospy.ServiceException, e:
      print 'clear costmaps failed'


  def stageCallback(self, msg):
    for node in self.nodes:
      stages = self.nodes[node]['stages']
      if msg.stage not in stages:
        continue

      if self.nodes[node]['stages'][msg.stage]['clear_costmap']:
        self.clear_costmaps()

      params = self.get_params(node, msg)

      # update params
      last_update = self.nodes[node].get('last_update', None)
      if params is not last_update:
        self.nodes[node]['last_update'] = params
        self.update(node, params)

    self.last_stage = msg

  def get_params(self, node, msg):
    stages = self.nodes[node]['stages']
    if msg.stage not in stages:
      return None
    stage = stages[msg.stage]

    if msg.backward_distance < stage['enter_distance']:
      return stage['enter']
    elif msg.forward_distance < stage['exit_distance']:
      return stage['exit']
    else:
      return stage['params']


if __name__ == '__main__':
  rospy.init_node('stage_reconfigure')
  stage = StageReconfigure()
  stage.load()
  rospy.spin()

