#!/usr/bin/env python3
import json
import threading
import rospy
from std_srvs.srv import Trigger
from battery_grasp.http_api import create_server


def main():
    rospy.init_node('api_control_server')
    host = rospy.get_param('~host','127.0.0.1')
    port = rospy.get_param('~port',18080)

    def backend(command):
        name = '/battery_grasp/'+command.lower()
        rospy.wait_for_service(name,timeout=2)
        result = rospy.ServiceProxy(name,Trigger)()
        return json.loads(result.message)

    server = create_server(host,port,backend)
    worker = threading.Thread(target=server.serve_forever,daemon=True)
    worker.start()
    rospy.on_shutdown(server.shutdown)
    rospy.loginfo('Simulation API listening at http://%s:%s',host,server.server_port)
    try:
        rospy.spin()
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=3)


if __name__=='__main__':
    main()
