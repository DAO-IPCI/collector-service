import threading
from flask import Flask
from flask_cors import CORS

import rospy
from std_srvs.srv import Empty
from collector_agent.srv import GetObjective, GetObjectiveRequest

threading.Thread(target=lambda: rospy.init_node(
    'collector_service_proxy', disable_signals=True)).start()

app = Flask(__name__)
CORS(app)


@app.route('/', methods=['POST'])
def call_demand_service():
    rospy.wait_for_service("publish_demand")
    publish_demand = rospy.ServiceProxy("publish_demand", Empty)
    rospy.loginfo("Calling the 'publish_demand' service")
    publish_demand()
    return ('', 204)


@app.route('/get_objective', methods=['POST'])
def call_get_objective_service():
    rospy.loginfo('Get objective')
    rospy.wait_for_service("get_objective")
    get_objective = rospy.ServiceProxy("get_objective", GetObjective)
    rospy.loginfo("Calling the 'get_objective' service")
    obj = get_objective(GetObjectiveRequest(''))
    return obj.objective


if __name__ == '__main__':
    app.run(port=8899)
