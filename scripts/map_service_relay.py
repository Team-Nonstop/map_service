#!/usr/bin/env python

import rospy
import json

from nav_msgs.msg import OccupancyGrid 
from nav_msgs.srv import GetMap,GetMapResponse,GetMapRequest
from concert_msgs.msg import *

class SubSrv(object):

    postfix = '_static' 
    map = None

    def __init__(self,id,namespace,topic_name):
        self.sub = rospy.Subscriber(topic_name,OccupancyGrid,self.process_map_sub)
        self.srv = rospy.Service(topic_name + self.postfix,GetMap,self.process_srv)
        self.map = OccupancyGrid()

    def process_srv(self,srv):
        return GetMapResponse(self.map)


    def process_map_sub(self,msg):
        self.map = msg

    def shutdown(self):
        self.srv.shutdown()
        self.sub.shutdown()

    

class MapRelayService(object):

    implmentation_topic = 'concert/implementation'
    map_type = 'nav_msgs/OccupancyGrid'
    services = {}
    
    def __init__(self):
        self.subscriber = {}
        self.subscriber['implmentation'] = rospy.Subscriber(self.implmentation_topic,Implementation,self.process_implementation)
    
    def process_implementation(self, msg):

        # New Implementation came, shutdown old services
        self.shutdown_services()

        # Get Map topic publisher edge
        edges = self.get_map_pub_edges(msg)

        # Starts Services
        self.start_services(edges)

    def start_services(self,edges):
        try:
        
            for e in edges:
                self.subscriber[e[0]] = SubSrv(e[0],e[1],e[2])
        except Exception as e:
            pass
            
    def get_map_pub_edges(self,msg):
        map_topics = [ t.id for t in msg.link_graph.topics if t.type == self.map_type]
            
        map_edges = [(e.finish, e.start, e.remap_to) for e in msg.link_graph.edges if e.finish in map_topics]

        return map_edges

    def shutdown_services(self):
        for s in self.services:
            s.shutdown()


        

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('map_relay_service')
    
    map_relay = MapRelayService()
    rospy.loginfo('Initialized')
    map_relay.spin()
    rospy.loginfo('Bye Bye')
